#!/usr/bin/env python3
"""Найти выход, которым включается ES01. Рука не движется.

Паспорт ES01 не фиксирует, на какой линии сидит захват: у E6 есть выходы на
контроллере (DO) и на фланце под инструмент (ToolDO). Скрипт перебирает их по
одному и спрашивает, слышно ли насос.

Ответ нужен дважды. Для управления захватом из кода — очевидно. И для прошивки:
оптопара PC817 ловит электрический фронт именно этой линии, чтобы поставить
t=0 в ту же шкалу, где лежат отсчёты давления.

Используются ТОЛЬКО мгновенные команды (DOInstant/ToolDOInstant): очередь
движений не трогается, EnableRobot не вызывается, суставы обесточены.

Запуск:
  tools/arm/find_es01.sh              # ToolDO 1-2, затем DO 1-8
  tools/arm/find_es01.sh --do-max 16  # если выходов больше
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor" / "dobot_tcpip_v4"))

from suction.arm.client import run_tool  # noqa: E402
from suction.config import ARM_IP, ARM_DASHBOARD_PORT, ARM_FEEDBACK_PORT  # noqa: E402

ON_SECONDS = 1.5


def err_id(resp):
    """Первое число ответа: 0 — команда принята."""
    try:
        return int(str(resp).split(",", 1)[0])
    except (ValueError, IndexError):
        return None


def value(resp):
    if not isinstance(resp, str) or "{" not in resp:
        return resp
    return resp[resp.index("{") + 1:resp.rindex("}")]


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        print("\nНужен интерактивный терминал: запусти из обычной консоли.")
        sys.exit(2)


def probe(io, kind, index):
    """Дёрнуть один выход. Возвращает 'found' / 'next' / 'stop'."""
    set_on = io.ToolDOInstant if kind == "ToolDO" else io.DOInstant
    get = io.GetToolDO if kind == "ToolDO" else io.GetDO

    was = value(get(index))
    name = f"{kind}{index}"

    ans = ask(f"\n  {name}: включаю на {ON_SECONDS:g} с. Enter — поехали, "
              f"s — пропустить, q — выход: ")
    if ans == "q":
        return "stop"
    if ans == "s":
        return "next"

    resp = set_on(index, 1)
    if err_id(resp) != 0:
        print(f"    контроллер отказал: {resp}")
        return "next"

    time.sleep(ON_SECONDS)
    set_on(index, 0)

    # вернуть как было, если выход изначально стоял в 1
    if str(was).strip() == "1":
        set_on(index, 1)
        print(f"    (выход был включён до нас — вернул в 1)")

    ans = ask("    насос работал? y/n: ")
    return "found" if ans.startswith(("y", "д")) else "next"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--do-max", type=int, default=8,
                    help="сколько выходов контроллера перебирать (по умолчанию 8)")
    ap.add_argument("--tool-max", type=int, default=2,
                    help="сколько выходов на фланце (по умолчанию 2)")
    args = ap.parse_args()

    print("=" * 62)
    print("ПОИСК ВЫХОДА ES01")
    print("Рука НЕ движется: питание на суставы не подаётся, очередь движений")
    print("не трогается. Щёлкает только реле выхода.")
    print()
    print("Перед стартом: убедись, что к выходам контроллера не подключено")
    print("ничего постороннего — перебор дёрнет их все по очереди.")
    print("=" * 62)
    if not ask("\nПоехали? y/n: ").startswith(("y", "д")):
        return 0

    from dobot_sdk import DobotRobot

    with DobotRobot(ARM_IP, dashboard_port=ARM_DASHBOARD_PORT,
                    feedback_port=ARM_FEEDBACK_PORT) as robot:
        ctl = robot.robot_control.RequestControl()
        if err_id(ctl) != 0:
            print(f"\nНе удалось взять TCP-режим: {ctl}")
            print("Обычно значит, что рука запитана или на паузе — контроллер")
            print("отдаёт управление только с обесточенных суставов.")
            return 1

        io = robot.io
        plan = ([("ToolDO", i) for i in range(1, args.tool_max + 1)]
                + [("DO", i) for i in range(1, args.do_max + 1)])

        print(f"\nФланец идёт первым: ES01 — инструментальный захват, вероятнее")
        print(f"всего он там. Всего проверок: {len(plan)}.")

        for kind, index in plan:
            result = probe(io, kind, index)
            if result == "found":
                print("\n" + "=" * 62)
                print(f"ES01 сидит на {kind}{index}")
                print("=" * 62)
                print("\nЗапиши в suction/config.py и в схему прошивки:")
                print(f"  ES01_OUTPUT_KIND = {kind!r}")
                print(f"  ES01_OUTPUT_INDEX = {index}")
                print("\nОптопару PC817 вешать на эту же линию — так метка t=0")
                print("окажется в одной шкале с отсчётами давления.")
                return 0
            if result == "stop":
                print("\nПрервано.")
                return 1

        print("\nНи один выход не дал звука. Что проверить:")
        print("  - ES01 запитан? Ему нужны 24 В, от логики выхода он не оживёт")
        print("  - разъём на фланце сидит плотно")
        print("  - выходов больше восьми: --do-max 16")
        print("  - захват может управляться не DO, а через SetToolMode/485")
    return 1


if __name__ == "__main__":
    sys.exit(run_tool(main))
