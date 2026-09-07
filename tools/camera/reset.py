#!/usr/bin/env python3
"""Программно передёрнуть USB камеры — замена выдёргиванию провода.

HP60C приходит в рабочее состояние не всегда. Если драйвер запускался поверх
не отпущенного как следует устройства, сенсор поднимается с мёртвой областью:
часть кадра приходит нулями ВСЕГДА, без единой ошибки в лог. Область не растёт
и не рассасывается — она фиксируется в момент открытия устройства и держится
до конца сессии. Лечится только переинициализацией на уровне шины.

Скрипт делает то же, что физическое переподключение: USBDEVFS_RESET на узле
устройства. Узел открыт на запись всем (crw-rw-rw-), root не нужен.

Порядок:
  1. остановить драйвер камеры (Ctrl+C в его окне, НЕ kill -9)
  2. tools/camera/reset.sh
  3. tools/camera/start.sh
  4. tools/camera/zone_check.sh   <- убедиться, что дыр нет

Запуск:
  tools/camera/reset.sh
"""
import platform
import re
import time
import subprocess
import sys
from pathlib import Path

VID_PID = "3482:6723"          # Nuwa/Angstrong HP60C
USBDEVFS_RESET = ord("U") << 8 | 20    # _IO('U', 20)


def find_node():
    try:
        out = subprocess.run(["lsusb", "-d", VID_PID], capture_output=True,
                             text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError) as e:
        print(f"lsusb не отработал: {e}")
        return None
    m = re.match(r"Bus (\d+) Device (\d+):", out.strip())
    if not m:
        return None
    return f"/dev/bus/usb/{m.group(1)}/{m.group(2)}"




def driver_running():
    """Живые процессы драйвера. Разбор /proc общий, см. tools/_lib/camera_proc.py."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "_lib"))
    from camera_proc import driver_pids
    launchers, nodes = driver_pids()
    return (launchers + nodes) or None


def wait_on_bus(timeout=15.0):
    """Дождаться, пока устройство вернётся на шину после сброса.

    USBDEVFS_RESET переподключает устройство: оно исчезает и появляется заново
    уже с ДРУГИМ номером в /dev/bus/usb. Кто ищет его сразу после сброса —
    не находит и решает, что камеру выдернули.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if find_node() is not None:
            return True
        time.sleep(0.5)
    return False


def main():
    if platform.system() != "Linux":
        print(f"Сброс USB через ioctl есть только в Linux, а тут "
              f"{platform.system()}.")
        print("Выдерни кабель камеры и воткни обратно — это ровно то же самое.")
        return 1
    import fcntl  # ядро Linux; на других системах модуля просто нет

    busy = driver_running()
    if busy:
        print(f"Драйвер камеры ещё работает (pid {', '.join(busy)}).")
        print()
        print("Сначала останови его — Ctrl+C в его окне, НЕ kill -9.")
        print("Сброс поверх работающего драйвера оставит устройство в худшем")
        print("состоянии, чем было.")
        return 1

    node = find_node()
    if node is None:
        print(f"Камера {VID_PID} на шине не найдена. Проверь, воткнута ли она.")
        return 1

    print(f"сбрасываю {node} ...")
    try:
        import fcntl
        with open(node, "wb") as f:
            fcntl.ioctl(f.fileno(), USBDEVFS_RESET, 0)
    except PermissionError:
        print(f"нет прав на {node}. Тогда остаётся выдернуть провод.")
        return 1
    except OSError as e:
        print(f"ioctl не прошёл: {e}. Выдерни провод.")
        return 1

    if not wait_on_bus():
        print("после сброса устройство не вернулось на шину за 15 с.")
        print("Выдерни кабель и воткни обратно.")
        return 1

    print("готово — устройство переинициализировано и снова на шине.")
    # Подсказку печатаем только при ручном запуске: из preflight драйвер
    # поднимается следующей же строкой, и совет запустить его сбивает с толку.
    if "--quiet" not in sys.argv:
        print()
        print("Дальше: tools/camera/start.sh, затем tools/camera/zone_check.sh.")
        print("Если дыры остались — значит дело не в шине, копай в драйвере.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
