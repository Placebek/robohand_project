#!/usr/bin/env python3
"""Первое движение E6. Мелкие повороты кисти и возврат в исходную позу.

Смысл не в том, чтобы рука поехала, а в трёх числах, которые нужны дальше:
сколько занимает разгон-торможение, с какой точностью она возвращается в ту же
позу и не сыплет ли контроллер ошибок на очередных командах. Повторяемость
позы — прямой вход в hand-eye: если рука не возвращается в ту же точку, невязка
AX=XB будет врать не из-за калибровки.

Позы берутся из потока состояния (порт 30004), поле q_actual. Именно так, а не
через GetAngle(): GetAngle отдаёт величину, привязанную к целевой, и после
возврата в ту же точку даёт расхождение ровно 0.0000° по всем шести суставам —
красиво и бессмысленно. q_actual идёт с энкодеров и показывает, что было на
самом деле.

По умолчанию крутится ТОЛЬКО J6 — вращение кисти, инструмент почти не меняет
положения в пространстве. Плечо (J1) — отдельным флагом, осознанно.

ЧЕЛОВЕК У АВАРИЙНОГО СТОПА ОБЯЗАТЕЛЕН.

Запуск:
  tools/arm/first_motion.sh                 # J6 ±10°, скорость 10%
  tools/arm/first_motion.sh --joint 1       # J1, шире размах в пространстве
  tools/arm/first_motion.sh --deg 5 --speed 5
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor" / "dobot_tcpip_v4"))

from suction.arm.client import run_tool  # noqa: E402
from suction.config import (ARM_IP, ARM_DASHBOARD_PORT, ARM_FEEDBACK_PORT,  # noqa: E402
                            ES01_MASS_KG)

MODE_ENABLED_IDLE = 5
MODE_ERROR = 9
SETTLE_TIMEOUT_S = 30.0


def err_id(resp):
    try:
        return int(str(resp).split(",", 1)[0])
    except (ValueError, IndexError):
        return None


def value(resp):
    if not isinstance(resp, str) or "{" not in resp:
        return str(resp)
    return resp[resp.index("{") + 1:resp.rindex("}")]


def angles(robot):
    """Реальные углы с энкодеров из потока состояния."""
    st = robot.GetStatus()
    if st is None:
        raise RuntimeError("поток состояния (30004) молчит")
    return list(st.joint_state.q_actual)


def mode(rc):
    try:
        return int(value(rc.RobotMode()))
    except ValueError:
        return None


def wait_idle(rc, timeout=SETTLE_TIMEOUT_S):
    """Дождаться, пока очередь опустеет. Возвращает секунды или None."""
    start = time.perf_counter()
    while time.perf_counter() - start < timeout:
        m = mode(rc)
        if m == MODE_ERROR:
            return None
        if m == MODE_ENABLED_IDLE and time.perf_counter() - start > 0.3:
            return time.perf_counter() - start
        time.sleep(0.05)
    return None


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        print("\nНужен интерактивный терминал: запусти из обычной консоли.")
        sys.exit(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", type=int, default=6, choices=range(1, 7),
                    help="какой сустав крутить (по умолчанию 6 — кисть)")
    ap.add_argument("--deg", type=float, default=10.0, help="размах в градусах")
    ap.add_argument("--speed", type=int, default=10, help="скорость, %% от полной")
    ap.add_argument("--cycles", type=int, default=3, help="сколько раз туда-обратно")
    args = ap.parse_args()

    print("=" * 64)
    print("ПЕРВОЕ ДВИЖЕНИЕ — РУКА ПОЕДЕТ")
    print("=" * 64)
    print(f"  сустав J{args.joint}, размах ±{args.deg:g}°, скорость {args.speed}%,"
          f" циклов {args.cycles}")
    print(f"  нагрузка для EnableRobot: {ES01_MASS_KG} кг (масса ES01)")
    print()
    print("  Перед стартом:")
    print("   - рука в свободном пространстве, вокруг ничего нет")
    print("   - кабель ES01 не натянут и не зацеплен")
    print("   - РУКА НА АВАРИЙНОМ СТОПЕ")
    print("=" * 64)
    if not ask("\nВсё убрано, держу стоп. Поехали? y/n: ").startswith(("y", "д")):
        return 0

    from dobot_sdk import DobotRobot

    with DobotRobot(ARM_IP, dashboard_port=ARM_DASHBOARD_PORT,
                    feedback_port=ARM_FEEDBACK_PORT) as robot:
        rc, motion = robot.robot_control, robot.motion
        enabled = False
        try:
            rc.RequestControl()
            rc.ClearError()
            robot.StartFeedbackMonitor()
            time.sleep(1.0)   # дать потоку набрать первый пакет

            print("\nзапитываю...")
            resp = rc.EnableRobot(load=ES01_MASS_KG)
            if err_id(resp) != 0:
                print(f"  отказ: {resp}")
                return 1
            enabled = True
            rc.SpeedFactor(args.speed)

            for _ in range(40):
                if mode(rc) == MODE_ENABLED_IDLE:
                    break
                time.sleep(0.1)
            print(f"  режим {mode(rc)} — запитана, простой")

            home = angles(robot)
            print(f"  исходные углы (q_actual): {['%.4f' % a for a in home]}")

            offsets = [0.0] * 6
            durations, returns = [], []

            for cycle in range(1, args.cycles + 1):
                print(f"\nцикл {cycle}/{args.cycles}")
                for sign in (+1, -1):
                    offsets[args.joint - 1] = sign * args.deg
                    t0 = time.perf_counter()
                    resp = motion.RelJointMovJ(*offsets)
                    if err_id(resp) != 0:
                        print(f"  команда отклонена: {resp}")
                        return 1
                    took = wait_idle(rc)
                    if took is None:
                        print(f"  движение не завершилось. ошибки: {rc.GetErrorID()}")
                        return 1
                    elapsed = time.perf_counter() - t0
                    durations.append(elapsed)
                    print(f"  {sign * args.deg:+g}° — {elapsed:.2f} с")

                back = angles(robot)
                drift = max(abs(b - h) for b, h in zip(back, home))
                returns.append(drift)
                print(f"  возврат: расхождение с исходной позой {drift:.4f}°")

            print("\n" + "=" * 64)
            print(f"движение туда-обратно: {min(durations):.2f}-{max(durations):.2f} с "
                  f"на {args.deg:g}° при {args.speed}%")
            print(f"повторяемость позы: худшее расхождение {max(returns):.4f}°")
            print(f"  (шум q_actual в покое — порядка 0.03°, ниже него не мерить)")
            if max(returns) < 0.05:
                print("  -> в пределах шума энкодера, для hand-eye годится")
            else:
                print("  -> расхождение выше шума, для hand-eye посмотреть внимательнее")
            print(f"ошибки контроллера: {value(rc.GetErrorID())}")
            print("=" * 64)

        except KeyboardInterrupt:
            print("\nCtrl+C — торможу.")
            try:
                rc.Stop()
            except Exception:
                pass
        finally:
            try:
                robot.StopFeedbackMonitor()
            except Exception:
                pass
            if enabled:
                print("обесточиваю суставы...")
                try:
                    rc.DisableRobot()
                except Exception as e:
                    print(f"  DisableRobot не прошёл: {e}. Сними питание вручную.")
    return 0


if __name__ == "__main__":
    sys.exit(run_tool(main))
