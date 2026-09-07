#!/usr/bin/env python3
"""Снять аварию с руки после срабатывания защиты. Ничего не двигает.

При ручном ведении легко дёрнуть руку резче, чем нравится контроллеру, и он
уходит в аварию (режим 9). Из этого состояния он не принимает ни движений, ни
повторного ведения, пока ошибку не сбросить.

Скрипт читает состояние, печатает коды, вызывает ClearError и показывает, что
получилось. Питание на суставы НЕ подаёт: после сброса рука остаётся
обесточенной, запитывает её уже тот скрипт, который будет работать дальше.

Если порт занят — значит другой скрипт ещё держит соединение. Закрой его
по-человечески (q или Ctrl+C), НЕ через kill -9: при штатном выходе он снимает
ручное ведение и обесточивает суставы.

Запуск:
  tools/arm/clear_fault.sh
"""
import json
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

from suction.arm.client import ArmClient, ArmError, run_tool  # noqa: E402

ALARM_FILES = [ROOT / "vendor" / "dobot_tcpip_v4" / "demo" / "files" / name
               for name in ("alarmController.json", "alarmServo.json")]


def alarm_table():
    """id -> (описание, что делать) из справочников Dobot, по-русски."""
    table = {}
    for path in ALARM_FILES:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        items = data if isinstance(data, list) else sum(
            (v for v in data.values() if isinstance(v, list)), [])
        for item in items:
            ru = item.get("ru") or item.get("en") or {}
            desc = ru.get("description") or ""
            if desc:
                table[int(item["id"])] = (desc, ru.get("solution", ""))
    return table


def explain(codes):
    if not codes:
        return
    table = alarm_table()
    print("  расшифровка:")
    for code in codes:
        try:
            desc, fix = table[int(code)]
        except (KeyError, ValueError):
            print(f"    {code}: в справочнике нет")
            continue
        print(f"    {code}: {desc}")
        if fix:
            print(f"          -> {fix}")


def report(arm, title):
    st = arm._status()
    print(f"{title}:")
    print(f"  режим {arm.mode()} — {arm.mode_name()}")
    print(f"  коды ошибок: {arm.errors() or 'нет'}")
    print(f"  error_status={st.error_status}  enable={st.enable_status}  "
          f"столкновение={st.collision_state}  safety={st.safety_state}")
    report = arm.error_report()
    if report:
        print(f"  контроллер сообщает:\n    {report}")
    else:
        explain(arm.errors())      # запасной путь: локальные таблицы
    return st


def main():
    try:
        arm = ArmClient().connect()
    except ArmError as e:
        if "occupied" in str(e) or "occupied" in repr(e):
            print("Порт 29999 занят другим скриптом.")
            print("Закрой его в его окне (q или Ctrl+C), НЕ через kill -9 —")
            print("при штатном выходе он снимает ведение и обесточивает суставы.")
            return 1
        raise

    with arm:
        before = report(arm, "до сброса")

        if arm.mode() != 9 and not before.error_status and not arm.errors():
            print("\nАварии нет — сбрасывать нечего.")
            return 0

        # Ручное ведение переживает аварию и мешает выйти из неё.
        try:
            arm._rc.StopDrag()
        except Exception:
            pass

        print("\nсбрасываю...")
        arm._rc.ClearError()
        time.sleep(0.5)

        after = report(arm, "\nпосле сброса")
        if after.error_status or arm.mode() == 9:
            print("\nАвария не снялась. Что бывает:")
            print("  - рука упёрлась в предел сустава: отведи её руками от упора")
            print("  - зажат аварийный стоп: отпусти и поверни грибок")
            print("  - защитная цепь разомкнута: проверь разъём E-STOP")
            print("  - что-то держит руку физически")
            print("Сдвинь руку и запусти сброс ещё раз.")
            return 1

        print("\nАвария снята, суставы обесточены.")
        print("Дальше запускай тот скрипт, который нужен — он запитает сам.")
    return 0


if __name__ == "__main__":
    sys.exit(run_tool(main))
