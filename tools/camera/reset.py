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
import fcntl
import re
import subprocess
import sys

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
        print(f"Камера {VID_PID} на шине не найдена. Проверь, воткнута ли она.")
        return None
    return f"/dev/bus/usb/{m.group(1)}/{m.group(2)}"


def driver_running():
    # -x по имени процесса, а не по всей командной строке: -f ловит любую
    # команду, где эта строка просто упомянута, включая собственную.
    pids = subprocess.run(["pgrep", "-x", "ascamera_node"],
                          capture_output=True, text=True).stdout.split()
    return pids or None


def main():
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
        return 1

    print(f"сбрасываю {node} ...")
    try:
        with open(node, "wb") as f:
            fcntl.ioctl(f.fileno(), USBDEVFS_RESET, 0)
    except PermissionError:
        print(f"нет прав на {node}. Тогда остаётся выдернуть провод.")
        return 1
    except OSError as e:
        print(f"ioctl не прошёл: {e}. Выдерни провод.")
        return 1

    print("готово — устройство переинициализировано.")
    print()
    print("Дальше: tools/camera/start.sh, затем tools/camera/zone_check.sh.")
    print("Если дыры остались — значит дело не в шине, копай в драйвере.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
