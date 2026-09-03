"""Единая точка правды по адресам, топикам и путям. Всё остальное импортирует отсюда."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA_RAW = ROOT / "data" / "raw"
DATA_CALIB = ROOT / "data" / "calib"
DATA_PROCESSED = ROOT / "data" / "processed"

# --- Manipulator Dobot Magician E6 ---------------------------------------
# Wi-Fi 192.168.1.6 | LAN1 192.168.5.1 | LAN2 192.168.100.1
# Подключение по RJ45 в LAN1: ноут держит статический 192.168.5.100/24
# (профиль NetworkManager "robohand-arm"), робот отвечает с 192.168.5.1.
ARM_IP = "192.168.5.1"
ARM_DASHBOARD_PORT = 29999   # команды
ARM_FEEDBACK_PORT = 30004    # поток состояния
ARM_PAYLOAD_KG = 0.75        # паспорт E6; сверься со своим экземпляром

# Захват ES01 сидит на инструментальном выходе фланца, НЕ на IO контроллера.
# Найдено перебором 2026-09-03 (tools/run_find_es01.sh): ToolDO1, первый же.
# Команда: robot.io.ToolDOInstant(ES01_OUTPUT_INDEX, 1). Мгновенная, проходит
# при обесточенных суставах — очередь движений не нужна.
ES01_OUTPUT_KIND = "ToolDO"
ES01_OUTPUT_INDEX = 1
ES01_MASS_KG = 0.25          # 250 г висит на фланце; это и есть load для EnableRobot

# --- Камера Nuwa HP60C (ROS 2, драйвер ascamera) --------------------------
RGB_TOPIC = "/ascamera_hp60c/camera_publisher/rgb0/image"
DEPTH_TOPIC = "/ascamera_hp60c/camera_publisher/depth0/image_raw"
DEPTH_INFO_TOPIC = "/ascamera_hp60c/camera_publisher/depth0/camera_info"
DEPTH_MIN_M = 0.2            # паспортный минимум — ближе камера не видит

# --- Вакуумный мост (Arduino) --------------------------------------------
VACUUM_PORT = "/dev/ttyUSB0"   # Nano; для ESP32 обычно /dev/ttyACM0
VACUUM_BAUD = 500000           # хватает на 100 Гц с запасом
PRESSURE_HZ = 100

# --- Пороги (перекалибровать после первых замеров, сейчас это заглушки) ---
SEAL_THRESHOLD_KPA = -30.0   # ниже — считаем уплотнение состоявшимся
LIFT_HOLD_S = 3.0            # сколько держать поднятым для метки success
