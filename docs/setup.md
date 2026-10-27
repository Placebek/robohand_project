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

## 7. Окружения для VLA (Трек А)

Запись демонстраций и обучение — разные машины и разные окружения; план
целиком — [vla_plan.md](vla_plan.md).

### 7.1. Почему на ноутбуке ДВА venv

`lerobot` нельзя ставить в основной `.venv` стенда. Проверено установкой
(`pip install --dry-run --report`), зависимости `lerobot 0.6.1`:

- `numpy>=2.0` — жёстко. В `.venv` сейчас numpy 1.26.4, и это не прихоть:
  `cv_bridge_boost.so` из ROS Jazzy собран под numpy 1.x (в бинарнике
  `numpy.core._multiarray_umath`, в numpy 2 переименован в `numpy._core`).
  Под numpy 2 cv_bridge не импортируется вообще — камеры нет.
- `opencv-python-headless>=4.9` — ровно то, что запрещено в CLAUDE.md:
  затеняет системный cv2 4.6.0, против которого собран cv_bridge. Плюс
  headless-сборка не умеет `imshow`, а окно OpenCV — весь интерфейс записи.

Поэтому окружения разведены, а вместе с ними — и шаги записи:

| venv | что в нём | чем пользуется |
|---|---|---|
| `.venv` (`--system-site-packages`) | ROS Jazzy, rclpy, cv_bridge, системный cv2 4.6.0, numpy 1.26 | вся съёмка и `tools/vla/record.sh` — пишет СЫРЫЕ эпизоды в `data/vla_raw/` |
| `.venv-lerobot` (изолированный) | lerobot 0.6.1, torch CPU, numpy 2.2, headless cv2 | только `tools/vla/pack.sh` — собирает `LeRobotDataset` из сырья |

Активировать `.venv-lerobot` в шелле и пойти работать со стендом — ошибка,
но не опасная: обёртки в `tools/` зовут интерпретатор `.venv` абсолютным
путём (`ROBOHAND_PY` в `tools/_lib/env.sh`) и снимают `VIRTUAL_ENV` у
дочерних процессов, так что активированное окружение ни на что не влияет.
До этой правки первый же прогон записи кончался segfault в `rclpy` после
`ImportError: compiled using NumPy 1.x` в `cv_bridge`.

Промежуточный формат описан в [`suction/vla/raw.py`](../suction/vla/raw.py).
Он нужен не только ради развязки: демонстрация — физический труд человека у
стенда, и терять серию из-за исключения в чужой библиотеке посреди записи
нельзя. Сырьё лежит на диске и перепаковывается сколько угодно раз.

### 7.2. Установка `.venv-lerobot` (ноутбук)

```bash
python3.12 -m venv .venv-lerobot          # БЕЗ --system-site-packages
.venv-lerobot/bin/pip install "torch==2.11.0+cpu" "torchvision==0.26.0+cpu" \
    --index-url https://download.pytorch.org/whl/cpu
.venv-lerobot/bin/pip install "lerobot[dataset]"
```

Две тонкости, обе стоили бы отладки:

- **`[dataset]`, а не голый `lerobot`.** `LeRobotDataset` требует `datasets`,
  `pandas`, `pyarrow`, `torchcodec`, `av`, `jsonlines` — все они лежат в extra
  `dataset`. Без него `pip install lerobot` встанет успешно, а датасет не
  запишется.
- **Торч ставить первым и с CPU-индекса, с пином `<2.12`.** Иначе pip тянет
  с PyPI полный CUDA-набор (`nvidia-*`, `triton`, `cuda-toolkit`, ~6 ГБ) на
  машину с Radeon 780M. Пин нужен потому, что `lerobot 0.6.1` требует
  `torch<2.12`, а по умолчанию CPU-индекс отдаёт 2.14 — pip тогда откатит его
  на совместимую версию уже с PyPI, то есть снова с CUDA.

Проверка, что окружение годное:
```bash
tools/vla/pack.sh --dry-run
```

Для видео нужен ffmpeg (в системе стоит 6.1.1) — `torchcodec` линкуется с ним.

### 7.3. Workstation для обучения

Обучение SmolVLA идёт не на ноутбуке (Radeon 780M, без CUDA), а на одной из
workstation с RTX 4070 Ti 12 ГБ — отдельная машина, отдельный venv:

```bash
python3.12 -m venv .venv-train && source .venv-train/bin/activate
pip install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu126
pip install "lerobot[smolvla,training]"
```
4070 Ti (Ada, CUDA 12.6/12.8) в списке поддерживаемых — Ampere/Ada/Hopper/
Blackwell при драйвере от 570.86. Ориентир по памяти: 10-16 ГБ VRAM на
batch size 8; на 12 ГБ работает с батчем поменьше или посадкой батча по факту
первого прогона (не гадать заранее, посмотреть `nvidia-smi` при обучении).

**Перенос датасета** ноутбук → workstation:
```bash
rsync -av data/vla_datasets/ workstation:~/robohand/data/vla_datasets/
```
Формат самодостаточен (Parquet + MP4 + JSON), конвертация не нужна. Сырьё
(`data/vla_raw/`) переносить не нужно — оно остаётся на ноутбуке как исходник
для перепаковки.
