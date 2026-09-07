#!/usr/bin/env python3
"""Единая точка запуска инструментов стенда. Работает и на Linux, и на Windows.

Зачем поверх обёрток .sh: обёртки решают ОДНУ задачу — вычищают snap-переменные
VS Code, которые ломают Qt-бэкенд OpenCV. Это чисто линуксовая беда, и на
Windows она не нужна, зато bash там нет. Поэтому:

  Linux   — запускается обёртка .sh, всё как раньше;
  Windows — скрипт запускается напрямую, без обёрток.

Что реально поедет на Windows: всё, что не трогает камеру. Съёмка кадров идёт
через ROS 2 и драйвер ascamera, собранные под Linux, и переносу не подлежит —
это ограничение поставщика камеры, а не проекта.

Примеры:
  python run.py                     список всего, что есть
  python run.py arm check
  python run.py calib solve
  python run.py measure pick-normal --approach
"""
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
LINUX = platform.system() == "Linux"

# группа -> имя -> (файл от корня, нужна ли камера через ROS, описание)
TOOLS = {
    "camera": {
        "start":       ("tools/camera/start.sh",     True,  "драйвер HP60C, останавливать ТОЛЬКО Ctrl+C"),
        "reset":       ("tools/camera/reset.py",     False, "сброс USB, лечит мёртвую область"),
        "zone-check":  ("tools/camera/zone_check.py", True, "что попадает в рабочий круг"),
        "depth-check": ("tools/camera/depth_check.py", True, "гейт по глубине"),
        "plan":        ("tools/camera/plan.py",      True,  "дистанция -> пятно, маркер, зона"),
        "rgbd":        ("tools/camera/rgbd.py",      True,  "просмотр RGB-D"),
    },
    "arm": {
        "check":        ("tools/arm/check.py",        False, "связь, режим, поза. Не двигает"),
        "clear-fault":  ("tools/arm/clear_fault.py",  False, "снять аварию"),
        "gripper":      ("tools/arm/gripper.py",      False, "присоска on / off / состояние"),
        "find-es01":    ("tools/arm/find_es01.py",    False, "поиск линии захвата"),
        "first-motion": ("tools/arm/first_motion.py", False, "ДВИЖЕНИЕ: J6, повторяемость"),
        "park":         ("tools/arm/park.py",         False, "ДВИЖЕНИЕ: парковка"),
    },
    "calib": {
        "make-board":   ("tools/calib/make_board.py",   False, "печатная ChArUco-доска"),
        "verify-board": ("tools/calib/verify_board.py", False, "проверка масштаба печати"),
        "capture":      ("tools/calib/capture.py",      True,  "съёмка поз, РУКА ПОДАТЛИВА"),
        "solve":        ("tools/calib/solve.py",        False, "решение AX=XB"),
        "check":        ("tools/calib/check.py",        True,  "сквозная проверка калибровки"),
    },
    "measure": {
        "pick-normal":  ("tools/measure/pick_normal.py",  True, "тычок -> нормаль"),
        "normal-error": ("tools/measure/normal_error.py", True, "ошибка нормали по кадру"),
    },
}


def show_list():
    print(__doc__.split("Примеры:")[0].strip())
    print(f"\nсистема: {platform.system()}\n")
    for group, items in TOOLS.items():
        print(f"{group}")
        for name, (path, needs_ros, doc) in items.items():
            mark = "  " if (LINUX or not needs_ros) else " *"
            print(f" {mark} {name:<14} {doc}")
    if not LINUX:
        print("\n* нужен ROS 2 и драйвер камеры — только Linux")


def main(argv):
    if len(argv) < 2:
        show_list()
        return 0
    group = argv[0]
    name = argv[1]
    if group not in TOOLS or name not in TOOLS[group]:
        print(f"нет такого инструмента: {group} {name}\n")
        show_list()
        return 2

    path, needs_ros, _ = TOOLS[group][name]
    rest = argv[2:]

    if needs_ros and not LINUX:
        print(f"«{group} {name}» работает только на Linux.")
        print("Съёмка кадров идёт через ROS 2 Jazzy и драйвер ascamera,")
        print("собранные под Linux. Это ограничение поставщика камеры.")
        print("\nБез камеры на этой машине доступны: arm (всё), calib solve,")
        print("calib make-board, calib verify-board.")
        return 1

    target = ROOT / path
    if LINUX:
        # На Linux идём через обёртку: в ней чистка snap-переменных VS Code,
        # без которой Qt-бэкенд OpenCV падает с symbol lookup error.
        wrapper = target.with_suffix(".sh")
        if wrapper.exists():
            return subprocess.call([str(wrapper), *rest])
        if target.suffix == ".sh":
            return subprocess.call([str(target), *rest])

    return subprocess.call([sys.executable, str(target), *rest])


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
