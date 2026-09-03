/*
 * Сенсорная приставка к заводской присоске DOBOT ES01.
 *
 * Конфигурация: ES01 включает сам робот по линии DO 24 В. Arduino ничем не
 * управляет — он ТОЛЬКО меряет. Две задачи:
 *   1) поток давления с датчика XGZP6847D (I2C) на максимальной частоте;
 *   2) захват электрического фронта команды DO робота на прерывании.
 *
 * Пункт 2 — не удобство, а необходимость. Метка «присоска включена», пришедшая
 * из контроллера робота по TCP, имеет джиттер в единицы миллисекунд — тот же
 * масштаб, что и измеряемый фронт нарастания разрежения. Захват фронта самим
 * Arduino ставит t=0 в ту же шкалу, где лежат отсчёты давления, и делает
 * синхронизацию аппаратной. Вопрос ревьюера «а точно ли ваш t=0 там, где вы
 * говорите» после этого закрывается одной фразой.
 *
 * Развязка: линия DO 24 В -> резистор 2.2к -> PC817 -> D2 (INT0), вход с
 * подтяжкой. Активное состояние DO открывает оптопару и тянет D2 к земле,
 * поэтому логика инвертирована (LOW = присоска включена).
 * ПЕРЕД СБОРКОЙ проверь мультиметром, DO робота выдаёт 24 В или замыкает на
 * землю (sourcing/sinking) — от этого зависит включение светодиода оптопары.
 *
 * Датчик (datasheet XGZP6847D V2.5, стр. 6-8):
 *   адрес 0x6D; запись 0x0A в рег. 0x30 -> совмещённое преобразование;
 *   ждать сброса бита Sco (0x08) в рег. 0x30; читать 0x06..0x08 -> 24 бита,
 *   старший бит знаковый: если ADC >= 2^23, то ADC -= 2^24;
 *   давление[Па] = ADC / K, для диапазона |P| <= 130 кПа K = 64.
 *
 * Протокол по USB (строки, \n):
 *   ->  T   тарировка: текущее давление принять за ноль (при открытой чашке)
 *       ?   информация
 *   <-  D <seq> <micros> <adc_raw> <do_state>   отсчёт давления
 *       E <micros> suction_on|suction_off       фронт DO робота
 *       # <text>                                комментарий/ошибка
 */

#include <Wire.h>

#define SENSOR_ADDR   0x6D
#define REG_CMD       0x30
#define REG_DATA_MSB  0x06
#define CMD_COMBINED  0x0A     // однократное совмещённое преобразование
#define SCO_BIT       0x08

// K для диапазона 65 < |P| <= 130 кПа (наш -100..0 кПа gauge). Датчик другого
// диапазона -> поменяй по таблице K в даташите, иначе шкала уедет в разы.
#define K_FACTOR      64.0f

const uint8_t PIN_DO_SENSE = 2;   // INT0, вход от PC817. LOW = ES01 включён
const uint32_t BAUD = 500000;

uint32_t seq = 0;
int32_t tare_adc = 0;

// Заполняется в прерывании, читается в loop -> volatile обязателен.
volatile uint32_t edge_us = 0;
volatile uint8_t edge_state = 0;
volatile bool edge_pending = false;

void onDoEdge() {
  edge_us = micros();
  edge_state = (digitalRead(PIN_DO_SENSE) == LOW) ? 1 : 0;   // инверсия оптопары
  edge_pending = true;
}

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

// --- Команды -------------------------------------------------------------
void handleCommand(const char *cmd) {
  switch (cmd[0]) {
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
      Serial.print(" do="); Serial.println(digitalRead(PIN_DO_SENSE) == LOW);
      break;
    default:
      Serial.println("# unknown command");
  }
}

void setup() {
  pinMode(PIN_DO_SENSE, INPUT_PULLUP);

  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);       // fast mode: на 100 кГц транзакции съедают бюджет цикла

  attachInterrupt(digitalPinToInterrupt(PIN_DO_SENSE), onDoEdge, CHANGE);

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
  // Фронт DO печатаем вне очереди: он и есть t=0 для всего анализа RQ1.
  if (edge_pending) {
    noInterrupts();
    uint32_t us = edge_us;
    uint8_t st = edge_state;
    edge_pending = false;
    interrupts();
    Serial.print("E "); Serial.print(us);
    Serial.println(st ? " suction_on" : " suction_off");
  }

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
  Serial.print(' '); Serial.println(digitalRead(PIN_DO_SENSE) == LOW ? 1 : 0);
}
