/*
 * Вакуумный мост: датчик CFSensor XGZP6847D (I2C) + насос + клапан 3/2.
 *
 * Зачем так: момент включения насоса метится ЧАСАМИ МК, в том же временном
 * базисе, что и отсчёты давления. Если ставить t=0 по времени хоста, USB-джиттер
 * (единицы миллисекунд) размажет фронт нарастания разрежения — а именно его
 * форма и есть главный признак для RQ1. Хост потом сшивает эту шкалу со своей
 * по строкам E (см. suction/vacuum/).
 *
 * Плата: Arduino Nano / ESP32. Для ESP32 поправь PIN_* и SDA/SCL.
 *
 * Протокол датчика (datasheet XGZP6847D V2.5, стр. 6-8):
 *   адрес 0x6D; запись 0x0A в рег. 0x30 -> совмещённое преобразование;
 *   ждать сброса бита Sco (0x08) в рег. 0x30; читать 0x06..0x08 -> 24 бита,
 *   старший бит - знак: если ADC >= 2^23, то ADC -= 2^24;
 *   давление[Па] = ADC / K, для диапазона -100..100 кПа K = 64.
 *
 * Протокол по USB (строки, \n):
 *   ->  P1 / P0     насос вкл/выкл
 *       V1 / V0     клапан вкл/выкл (вкл = вакуум на чашку, выкл = сброс в атмосферу)
 *       T           тарировка: текущее давление принять за 0 (делать при открытой чашке)
 *       ?           информация
 *   <-  D <seq> <micros> <adc_raw> <pump> <valve>    отсчёт давления
 *       E <micros> <event>                            событие (pump_on, valve_off, ...)
 *       # <text>                                      комментарий/ошибка
 */

#include <Wire.h>

#define SENSOR_ADDR   0x6D
#define REG_CMD       0x30
#define REG_DATA_MSB  0x06
#define CMD_COMBINED  0x0A     // однократное совмещённое преобразование
#define SCO_BIT       0x08

// K для диапазона 65 < |P| <= 130 кПа (наш -100..0 кПа gauge). Датчик на другой
// диапазон -> поменяй по таблице K в даташите, иначе шкала уедет в разы.
#define K_FACTOR      64.0f

const uint8_t PIN_PUMP  = 5;   // -> затвор IRLZ44N (насос)
const uint8_t PIN_VALVE = 6;   // -> затвор IRLZ44N (клапан 3/2)

const uint32_t BAUD = 500000;

uint32_t seq = 0;
int32_t tare_adc = 0;
bool pump_on = false;
bool valve_on = false;

// --- I2C -----------------------------------------------------------------
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)SENSOR_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Возвращает знаковый 24-битный ADC, или INT32_MIN при ошибке.
int32_t readPressureAdc() {
  if (!writeReg(REG_CMD, CMD_COMBINED)) return INT32_MIN;

  // Опрос Sco вместо глухой задержки 20 мс: типовое преобразование ~2-3 мс,
  // это и есть разница между 100 Гц и 50 Гц.
  uint8_t status = 0;
  uint32_t t0 = micros();
  do {
    if (!readRegs(REG_CMD, &status, 1)) return INT32_MIN;
    if (micros() - t0 > 40000UL) return INT32_MIN;   // 40 мс - таймаут
  } while (status & SCO_BIT);

  uint8_t b[3];
  if (!readRegs(REG_DATA_MSB, b, 3)) return INT32_MIN;

  int32_t adc = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2];
  if (adc & 0x800000L) adc -= 0x1000000L;            // старший бит - знак
  return adc;
}

// --- Управление ----------------------------------------------------------
void setPump(bool on) {
  pump_on = on;
  digitalWrite(PIN_PUMP, on ? HIGH : LOW);
  Serial.print("E "); Serial.print(micros());
  Serial.println(on ? " pump_on" : " pump_off");
}

void setValve(bool on) {
  valve_on = on;
  digitalWrite(PIN_VALVE, on ? HIGH : LOW);
  Serial.print("E "); Serial.print(micros());
  Serial.println(on ? " valve_on" : " valve_off");
}

void handleCommand(const char *cmd) {
  switch (cmd[0]) {
    case 'P': setPump(cmd[1] == '1'); break;
    case 'V': setValve(cmd[1] == '1'); break;
    case 'T': {
      int32_t adc = readPressureAdc();
      if (adc != INT32_MIN) {
        tare_adc = adc;
        Serial.print("# tare_adc="); Serial.println(tare_adc);
      } else {
        Serial.println("# tare failed: sensor not responding");
      }
      break;
    }
    case '?':
      Serial.print("# xgzp6847d K="); Serial.print(K_FACTOR);
      Serial.print(" tare_adc="); Serial.print(tare_adc);
      Serial.print(" pump="); Serial.print(pump_on);
      Serial.print(" valve="); Serial.println(valve_on);
      break;
    default:
      Serial.println("# unknown command");
  }
}

void setup() {
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_VALVE, LOW);

  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);       // fast mode: на 100 кГц транзакции съедают бюджет цикла

  delay(50);
  int32_t adc = readPressureAdc();
  if (adc == INT32_MIN) {
    Serial.println("# ERROR: XGZP6847D не отвечает. Проверь SDA/SCL, питание, подтяжки 4.7k");
  } else {
    tare_adc = adc;
    Serial.print("# ready, tare_adc="); Serial.println(tare_adc);
  }
}

void loop() {
  static char buf[16];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; handleCommand(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  int32_t adc = readPressureAdc();
  uint32_t t = micros();
  if (adc == INT32_MIN) {
    Serial.println("# read failed");
    delay(10);
    return;
  }

  // Сырой ADC, а не паскали: пересчёт и тарировку делает хост, чтобы можно было
  // перекалибровать уже собранный датасет, не перезаписывая его.
  Serial.print("D "); Serial.print(seq++);
  Serial.print(' '); Serial.print(t);
  Serial.print(' '); Serial.print(adc);
  Serial.print(' '); Serial.print(pump_on ? 1 : 0);
  Serial.print(' '); Serial.println(valve_on ? 1 : 0);
}
