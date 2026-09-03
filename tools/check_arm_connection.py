#!/usr/bin/env python3
"""Проверка связи с Dobot Magician E6. Рука не запитывается и не движется.

Скрипт читает порты, режим, ошибки и текущую позу. EnableRobot не вызывается,
команд движения нет — сервоприводы остаются обесточенными.

Одно исключение из «только чтение»: RequestControl(). Робот не отвечает ни на
один запрос, пока управление не переключено в TCP-режим, поэтому переключение
неизбежно. Оно не подаёт питание на суставы и не двигает рукой, но забирает
управление у пульта или DobotStudio — если там открыт проект, он потеряет
связь. Обратно переключается там же, откуда забрали.

Запуск:
  tools/run_arm_check.sh
"""
import socket
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor" / "dobot_tcpip_v4"))

from suction.config import ARM_IP, ARM_DASHBOARD_PORT, ARM_FEEDBACK_PORT  # noqa: E402

MODES = {
    0: "неизвестен", 1: "инициализация", 2: "тормоз отпущен",
    3: "обесточен", 4: "не запитан (disabled)", 5: "запитан, простой",
    6: "режим ручного ведения", 7: "выполняет программу",
    8: "одиночное движение", 9: "АВАРИЯ", 10: "пауза", 11: "джог",
}


def port_open(host, port, timeout=2.0):
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def value(resp):
    """Из ответа вида '0,{...},GetPose();' достать середину."""
    if not isinstance(resp, str) or "{" not in resp:
        return resp
    return resp[resp.index("{") + 1:resp.rindex("}")]


def main():
    print(f"Робот: {ARM_IP}")

    ports = {
        ARM_DASHBOARD_PORT: "команды (dashboard)",
        ARM_FEEDBACK_PORT: "поток состояния",
    }
    for port, what in ports.items():
        ok = port_open(ARM_IP, port)
        print(f"  порт {port} — {what}: {'открыт' if ok else 'НЕ ОТВЕЧАЕТ'}")
        if not ok:
            print("\nНет связи. Проверь по порядку:")
            print("  1. Кабель в LAN1 робота, светодиоды на разъёме горят.")
            print("  2. ip -brief addr — на ethernet должен быть 192.168.5.100/24")
            print("     (профиль: nmcli con up robohand-arm)")
            print(f"  3. ping {ARM_IP}")
            return 1

    from dobot_sdk import DobotRobot

    with DobotRobot(ARM_IP, dashboard_port=ARM_DASHBOARD_PORT,
                    feedback_port=ARM_FEEDBACK_PORT) as robot:
        rc = robot.robot_control

        # Без TCP-режима любой запрос возвращает "Control Mode Is Not Tcp".
        # Питание на суставы это не подаёт.
        ctl = rc.RequestControl()
        print(f"\nпереключение в TCP-режим: {ctl}")

        mode_raw = rc.RobotMode()
        try:
            mode = int(value(mode_raw))
        except (TypeError, ValueError):
            mode = None
        print(f"\nрежим: {mode} — {MODES.get(mode, '?')}"
              if mode is not None else f"\nрежим: {mode_raw}")

        err = value(rc.GetErrorID())
        clean = err.replace("[", "").replace("]", "").replace(",", "").strip()
        print(f"ошибки: {'нет' if not clean or clean == '0' else err}")

        pose = value(rc.GetPose())
        angle = value(rc.GetAngle())
        print(f"\nпоза XYZ+RPY: {pose}")
        print(f"углы суставов: {angle}")

    print("\nСвязь есть, рука не тронута.")
    print("Следующий шаг — запитать (EnableRobot) и подвигать; это отдельный")
    print("скрипт и делается при человеке рядом с кнопкой аварийного стопа.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
