#!/usr/bin/env python3
"""Запись ручных поз и их воспроизведение для Dobot Magician E6 + ES01.

Обучение: рука запитывается, включается hand-guiding; оператор физически
перемещает манипулятор. Enter сохраняет текущую позу, on/off сохраняют событие
вакуума в текущей позе, done завершает запись.

Воспроизведение по умолчанию не запускается автоматически: требуется явное
подтверждение PLAY. Перед первым запуском уберите предметы из рабочей зоны.
"""
import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

from suction.arm.client import ArmClient, ArmError, run_tool  # noqa: E402
from suction.config import ES01_MASS_KG  # noqa: E402

DEFAULT_PROGRAM = ROOT / "programs" / "pick_and_place.json"
SCHEMA_VERSION = 1


def _finite_vector(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label}: ожидался список из {length} чисел")
    result = []
    for item in value:
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{label}: найдено нечисловое или бесконечное значение")
        result.append(number)
    return result


def validate_program(data):
    """Проверить файл до подключения к движению; возвращает список точек."""
    if not isinstance(data, dict) or data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"неподдерживаемая версия программы (нужна {SCHEMA_VERSION})")
    points = data.get("points")
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("программа должна содержать минимум две точки")
    checked = []
    for i, point in enumerate(points):
        if not isinstance(point, dict):
            raise ValueError(f"точка {i + 1}: неверный формат")
        name = str(point.get("name", "")).strip()
        if not name:
            raise ValueError(f"точка {i + 1}: пустое имя")
        joints = _finite_vector(point.get("joints"), 6, f"{name}.joints")
        pose = _finite_vector(point.get("pose"), 6, f"{name}.pose")
        vacuum = point.get("vacuum")
        if not isinstance(vacuum, bool):
            raise ValueError(f"{name}.vacuum должен быть true/false")
        checked.append({"name": name, "joints": joints, "pose": pose,
                        "vacuum": vacuum})
    return checked


def load_program(path):
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except OSError as e:
        raise ValueError(f"не удалось прочитать {path}: {e}") from e
    except json.JSONDecodeError as e:
        raise ValueError(f"повреждён JSON {path}: {e}") from e
    return validate_program(data)


def save_program(path, points):
    data = {
        "schema_version": SCHEMA_VERSION,
        "robot": "Dobot Magician E6",
        "tool": "DOBOT ES01",
        "coordinate_source": "q_actual + tool_vector_actual from feedback port 30004",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "motion_type": "joint",
        "points": points,
    }
    validate_program(data)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                         encoding="utf-8")
    temporary.replace(path)


def capture_point(arm, name, vacuum):
    return {
        "name": name,
        "joints": [round(float(x), 5) for x in arm.joints()],
        "pose": [round(float(x), 4) for x in arm.pose()],
        "vacuum": bool(vacuum),
    }


def teach(arm, output, speed):
    print("\nРЕЖИМ ОБУЧЕНИЯ")
    print("Робот станет податливым. Поддерживай руку при включении ручного ведения.")
    print("Команды: Enter — сохранить позу; on/off — переключить вакуум и")
    print("         сохранить событие в текущей позе; done — завершить.")
    if input("Убедись, что присоска ничего не держит. Введи EMPTY: ").strip() != "EMPTY":
        print("Отменено.")
        return 0
    if input("Начать обучение? введи TEACH: ").strip() != "TEACH":
        print("Отменено.")
        return 0

    arm.gripper(False)
    arm.enable(load_kg=ES01_MASS_KG)
    arm.speed(speed)
    points = []
    vacuum = arm.gripper_state()
    drag_started = False
    try:
        arm.drag(True)
        drag_started = True
        print("Ручное ведение включено. Перемещай руку физически.")
        while True:
            command = input(f"[точек: {len(points)}, вакуум: {'ON' if vacuum else 'OFF'}] "
                            "Enter/on/off/done: ").strip().lower()
            if command in ("done", "q", "quit"):
                break
            if command == "on" or command == "off":
                new_state = command == "on"
                arm.gripper(new_state)
                vacuum = new_state
                name = input("Имя события (например pick_grip): ").strip()
                if not name:
                    name = f"point_{len(points) + 1:03d}"
                points.append(capture_point(arm, name, vacuum))
                print(f"Сохранено событие {name}: вакуум {'ON' if vacuum else 'OFF'}")
                continue
            if command:
                print("Неизвестная команда. Используй Enter, on, off или done.")
                continue
            name = input("Имя точки (например pick_approach): ").strip()
            if not name:
                name = f"point_{len(points) + 1:03d}"
            points.append(capture_point(arm, name, vacuum))
            print(f"Сохранено: {name}; суставы={points[-1]['joints']}")
    finally:
        if drag_started:
            try:
                arm.drag(False)
            except Exception as e:
                print(f"Предупреждение: не удалось выключить ручное ведение: {e}")

    if len(points) < 2:
        print("Запись отменена: сохранено меньше двух точек.")
        return 1
    save_program(output, points)
    print(f"\nСохранено {len(points)} точек: {output}")
    print("Перед воспроизведением проверь порядок точек и события вакуума в JSON.")
    return 0


def show_program(points):
    print("\nПоследовательность:")
    for i, p in enumerate(points, 1):
        state = "ON" if p["vacuum"] else "OFF"
        print(f" {i:02d}. {p['name']:<24} vacuum={state} joints={p['joints']}")


def playback(arm, path, speed, reverse=False):
    points = load_program(path)
    if reverse:
        points = list(reversed(points))
        print("ВНИМАНИЕ: обратное воспроизведение меняет порядок точек, но не")
        print("переворачивает состояния вакуума. Проверь JSON и расположение предмета.")
    show_program(points)
    print("\nПеред запуском убедись, что рабочая зона свободна, кроме нужного предмета,")
    print("и первая точка программы соответствует безопасному стартовому положению.")
    if input("Для запуска введи PLAY: ").strip() != "PLAY":
        print("Отменено.")
        return 0

    arm.enable(load_kg=ES01_MASS_KG)
    arm.speed(speed)
    previous_vacuum = arm.gripper_state()
    try:
        for i, point in enumerate(points, 1):
            if arm.mode() == 9 or arm.errors():
                raise ArmError(f"авария до точки {point['name']}: {arm.errors()}")
            # Проверка достижимости выполняется до команды движения.
            if not arm.reachable(point["pose"]):
                raise ArmError(f"точка {point['name']} не проходит проверку достижимости")
            print(f"[{i}/{len(points)}] -> {point['name']}")
            arm.move_joints(point["joints"])
            if point["vacuum"] != previous_vacuum:
                arm.gripper(point["vacuum"])
                previous_vacuum = point["vacuum"]
                print(f"  вакуум {'ON' if previous_vacuum else 'OFF'}")
        print("Последовательность завершена.")
    except KeyboardInterrupt:
        print("\nПрерывание: отправляю команду остановки.")
        try:
            arm.stop()
        finally:
            raise
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("teach", "play", "reverse", "show"))
    parser.add_argument("--file", type=Path, default=DEFAULT_PROGRAM,
                        help=f"файл программы (по умолчанию {DEFAULT_PROGRAM})")
    parser.add_argument("--speed", type=int, default=10, help="скорость робота, 1-100%%")
    args = parser.parse_args()
    if not 1 <= args.speed <= 100:
        parser.error("--speed должен быть от 1 до 100")

    if args.mode == "show":
        show_program(load_program(args.file))
        return 0

    if args.mode in ("play", "reverse"):
        # Сначала валидируем файл до установления связи с роботом.
        load_program(args.file)

    with ArmClient() as arm:
        print(f"Dobot E6: режим {arm.mode_name()}, суставы={arm.joints()}")
        if args.mode == "teach":
            return teach(arm, args.file, args.speed)
        return playback(arm, args.file, args.speed, reverse=args.mode == "reverse")


if __name__ == "__main__":
    sys.exit(run_tool(main))
