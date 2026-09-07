#!/usr/bin/env python3
"""Увести руку в парковочную позу и обесточить. Запускать перед уходом.

У E6 абсолютные энкодеры, процедуры homing в TCP-протоколе нет — «домой»
здесь означает просто прийти в заранее известную позу и снять питание.

Движение идёт в СУСТАВНЫХ координатах, а не декартовых. Причина: у сложенной
конфигурации кисть вырождена (J5 около нуля), и обе кинематики, прямая и
обратная, отвечают на неё -2. Суставное движение кинематику не спрашивает.

Что делает по порядку:
  1. снимает аварию, если она висит;
  2. выключает присоску — иначе она гудит всю ночь;
  3. запитывает, уводит в PARK_JOINTS на малой скорости;
  4. обесточивает суставы.

РУКА ПОЕДЕТ. Смотри на неё, аварийный стоп под рукой.

Запуск:
  tools/arm/park.sh              # в парковочную позу и обесточить
  tools/arm/park.sh --here       # никуда не ехать, только выключить всё
"""
import argparse
import sys
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

from suction.arm.client import ArmClient, ArmError, run_tool  # noqa: E402
from suction.config import PARK_JOINTS  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--here", action="store_true",
                    help="не двигать: только выключить присоску и обесточить")
    ap.add_argument("--speed", type=int, default=10, help="скорость, %%")
    ap.add_argument("--joints", type=float, nargs=6, default=None,
                    help="куда парковать, шесть углов вместо PARK_JOINTS")
    args = ap.parse_args()

    target = list(args.joints) if args.joints else list(PARK_JOINTS)

    with ArmClient() as arm:
        print(f"режим {arm.mode()} — {arm.mode_name()}")
        here = arm.joints()
        print(f"углы сейчас: {['%.2f' % a for a in here]}")

        if arm.errors() or arm.mode() == 9:
            print(f"висит авария: {arm.errors()}")
            report = arm.error_report()
            if report:
                print(f"  {report}")
            print("снимаю...")
            arm._rc.ClearError()

        if arm.gripper_state():
            print("выключаю присоску (придержи, если на ней что-то висит)")
            arm.gripper(False)

        if args.here:
            print("--here: никуда не еду.")
        else:
            delta = max(abs(a - b) for a, b in zip(here, target))
            print(f"\nпарковка: {['%.2f' % a for a in target]}")
            print(f"максимальный поворот сустава: {delta:.1f}°")
            print("РУКА ПОЕДЕТ. Аварийный стоп под рукой.")
            if input("ехать? y/n: ").strip().lower() not in ("y", "д", "да"):
                print("отменено, суставы оставляю как есть")
                return 0
            arm.enable()
            arm.speed(args.speed)
            try:
                took = arm.move_joints(target)
                print(f"приехали за {took:.1f} с")
            except ArmError as e:
                print(f"движение не прошло: {e}")
                report = arm.error_report()
                if report:
                    print(f"  контроллер: {report}")
                print("суставы обесточу всё равно")

        # выход из with сам вызовет disable(), но скажем об этом вслух
        print("обесточиваю суставы")
    print("\nГотово. Питание контроллера и камеру выключай как обычно.")
    return 0


if __name__ == "__main__":
    sys.exit(run_tool(main))
