# Установка окружения

## 1. Что уже стоит (проверено на этой машине)

- Ubuntu 24.04, ROS 2 **Jazzy** в `/opt/ros/jazzy`
- Python 3.12.3, cv2 **4.6.0** (системный, с `aruco` и `calibrateHandEye`)
- numpy 1.26.4, scipy, pandas, matplotlib, pyserial
- Драйвер камеры HP60C собран:
  `~/Downloads/4.ROS2_Code-20260115T112805Z-3-001/4.ROS2_Code/ascam_ros2_ws`
- Пользователь в группе `dialout` (Arduino заведётся без sudo)

## 2. Что скачано в проект

| Куда | Что | Откуда |
|---|---|---|
| `vendor/dobot_tcpip_v4/` | Dobot TCP/IP Python SDK V4 (поддерживает E6) | `github.com/Dobot-Arm/TCP-IP-Python-V4` |
| `.venv/` | venv с `--system-site-packages` | — |

`--system-site-packages` принципиально: без него из venv не видны `rclpy` и
`cv_bridge`, а они нужны для камеры. По той же причине **не ставь
`opencv-python` в venv** — он затенит системный cv2 4.6.0, против которого
скомпилирован `cv_bridge`, и камера отвалится.

## 3. Ещё предстоит скачать

### Arduino toolchain (нужен для прошивки датчика)
```bash
sudo apt install arduino-cli   # или: curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
arduino-cli core install arduino:avr        # для Nano
arduino-cli core install esp32:esp32        # для ESP32
```

### Библиотека датчика
XGZP6847D по I2C — регистровый протокол простой, драйвер написан вручную в
`firmware/suction_sensor/` (внешняя библиотека не нужна).
Если возьмёшь MPRLS вместо него:
```bash
arduino-cli lib install "Adafruit MPRLS Library"
```

### PyTorch — только на этапе 4, если бустинг упрётся в потолок
```bash
.venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
```
CPU-версия: ряды давления короткие, GPU не нужен. Экономит ~2 ГБ.

### Литература (положить в `docs/refs/`)
- Mahler et al., **Dex-Net 3.0** — arXiv:1709.06670 — главная точка отсчёта по планированию присасывания
- **VacuumVLA** — исходная статья, чей Limitations §2 мы и закрываем

## 4. Место на диске

Свободно ~18 ГБ на `/`. Датасет фазы 1 (~2000 попыток × RGB-D кадр) — оценочно
15–25 ГБ. **Пиши `data/raw` на внешний Netac Z6s**, иначе упрёшься в диск на
середине сбора:
```bash
ln -s /media/jake/<SSD>/robohand_data data/raw
```

## 5. Сеть до робота

На ноуте **нет RJ45** (только `wlp1s0`). Варианты:
- Wi-Fi робота: `192.168.1.6`
- USB-Ethernet адаптер: LAN1 `192.168.5.1`, LAN2 `192.168.100.1`

Порты: **29999** — команды (dashboard), **30004** — поток состояния.
Робот должен быть в режиме удалённого TCP/IP управления (не в режиме DobotStudio).

Проверка: `ping 192.168.1.6` → сейчас не отвечает (робот не подключён).

## 6. Восстановление на чистой машине

`vendor/` и `.venv/` в git не лежат (см. `.gitignore`) — после `git clone` их надо
поднять заново:

```bash
git clone https://github.com/Placebek/robohand_project.git robohand && cd robohand
git clone --depth 1 https://github.com/Dobot-Arm/TCP-IP-Python-V4.git vendor/dobot_tcpip_v4
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt -e vendor/dobot_tcpip_v4
```

Это же — часть заявленного вклада «открытый воспроизводимый стенд» (план §11):
если репозиторий не разворачивается с нуля одной командой, ценность открытого
кода для рецензента околонулевая.
