#!/usr/bin/env python3
"""Включить, выключить или спросить состояние присоски ES01.

Захват сидит на ToolDO1 — выход на фланце, не на клеммнике контроллера.
Команда мгновенная и проходит при обесточенных суставах, так что рука для
этого не запитывается и не движется.

Запуск:
  tools/arm/gripper.sh on      # включить вакуум
  tools/arm/gripper.sh off     # выключить
  tools/arm/gripper.sh         # только спросить состояние
"""
import sys
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

from suction.arm.client import ArmClient, run_tool  # noqa: E402


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
    sys.exit(run_tool(main))
