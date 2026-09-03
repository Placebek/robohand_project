#!/usr/bin/env python3
"""Включить, выключить или спросить состояние присоски ES01.

Захват сидит на ToolDO1 — выход на фланце, не на клеммнике контроллера.
Команда мгновенная и проходит при обесточенных суставах, так что рука для
этого не запитывается и не движется.

Запуск:
  tools/run_gripper.sh on      # включить вакуум
  tools/run_gripper.sh off     # выключить
  tools/run_gripper.sh         # только спросить состояние
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from suction.arm.client import ArmClient  # noqa: E402


def main():
    arg = (sys.argv[1].lower() if len(sys.argv) > 1 else "state")
    if arg not in ("on", "off", "state"):
        print(__doc__)
        return 2

    with ArmClient() as arm:
        if arg != "state":
            arm.gripper(arg == "on")
        state = arm.gripper_state()
        print(f"присоска: {'ВКЛЮЧЕНА' if state else 'выключена'}")
        if arg == "off" and not state:
            print("если доска висела на присоске — она уже упала, проверь")
    return 0


if __name__ == "__main__":
    sys.exit(main())
