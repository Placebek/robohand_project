#!/usr/bin/env python3
"""Тычешь в точку на картинке — получаешь нормаль поверхности в координатах руки.

Это ядро RQ2. Цепочка целиком: пиксель -> глубина -> облако точек в окрестности
размером с чашку -> подгонка плоскости -> нормаль -> перевод в базу робота ->
поза, в которой чашка встанет параллельно поверхности.

Окрестность берётся РАЗМЕРОМ С ЧАШКУ (35 мм), а не фиксированным числом
пикселей: чашка уплотняется по всему своему пятну, и нормаль имеет смысл
только на этом масштабе. В пикселях этот радиус зависит от дистанции, поэтому
считается на лету из глубины в самой точке.

Точка обязана лежать в рабочем круге r<180 px вокруг главной точки: дальше
карта глубины радиально выгибается, и наклон плоскости будет измерен неверно.
Скрипт это проверяет и отказывается считать снаружи.

По умолчанию НИЧЕГО НЕ ДВИГАЕТ — только считает и печатает. Подвод руки
включается флагом --approach и требует отдельного подтверждения на каждую
точку.

Захват целиком (--pick): подвод по нормали, вакуум, посадка с
продавливанием чашки, подъём. Держит или нет — видно только глазами:
обратной связи по разрежению пока нет, она и есть предмет работы.

Запуск:
  tools/measure/pick_normal.sh              # только считать
  tools/measure/pick_normal.sh --approach   # ещё и подводить руку
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

import cv2                      # noqa: E402
import numpy as np              # noqa: E402
import rclpy                    # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node     # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

from suction.calib.handeye import load_handeye  # noqa: E402
from suction.arm.client import run_tool  # noqa: E402
from suction.config import (CUP_DIAMETER_MM, DATA_CALIB, DEPTH_TOPIC,  # noqa: E402
                            RGB_TOPIC, RGB_INFO_TOPIC, TOOL_LENGTH_MM,
                            TOOL_OFFSET_MM, WORK_CENTRE_PX, WORK_RADIUS_PX)

# ВНИМАНИЕ: только латиница. Qt-бэкенд OpenCV 4.6 не находит окна с кириллицей
# в названии, и setMouseCallback падает с «NULL window handler». Проверено:
# 'pick' привязывается сразу, любое русское имя — никогда.
WINDOW = "pick normal - klik v tochku, q vyhod"
MIN_PATCH_PX = 20           # меньше точек — плоскость не подогнать
MAX_RESIDUAL_MM = 3.0       # выше — это не плоскость, а край или дыра
MIN_LIFT_MM = 30.0          # ниже этого подъём бессмысленен: предмет не оторван


class Scene(Node):
    def __init__(self):
        super().__init__("pick_normal")
        self.b = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None
        self.create_subscription(Image, RGB_TOPIC,
                                 lambda m: setattr(self, "rgb",
                                                   self.b.imgmsg_to_cv2(m, "bgr8")), 10)
        self.create_subscription(Image, DEPTH_TOPIC,
                                 lambda m: setattr(self, "depth",
                                                   self.b.imgmsg_to_cv2(m, "passthrough")), 10)
        self.create_subscription(CameraInfo, RGB_INFO_TOPIC,
                                 lambda m: setattr(self, "K",
                                                   np.array(m.k).reshape(3, 3)), 10)

    def wait_ready(self, timeout=10.0):
        end = time.monotonic() + timeout
        # not "None in (...)": на массивах numpy это поэлементное сравнение
        while any(x is None for x in (self.rgb, self.depth, self.K)):
            if time.monotonic() > end:
                raise RuntimeError("нет кадров или интринсик — драйвер камеры запущен?")
            rclpy.spin_once(self, timeout_sec=0.1)

    def pump(self):
        rclpy.spin_once(self, timeout_sec=0.01)


def estimate_normal(depth, K, u, v):
    """Плоскость в окрестности размером с чашку. Возвращает словарь или причину отказа."""
    h, w = depth.shape
    z0 = float(depth[v, u])
    if z0 <= 0:
        return {"ошибка": "в этой точке глубины нет (ноль)"}
    z0_m = z0 / 1000.0

    # радиус чашки, переведённый в пиксели на этой дистанции
    rad_px = max(4.0, (CUP_DIAMETER_MM / 2.0) * K[0, 0] / z0)

    y0, y1 = max(0, int(v - rad_px)), min(h, int(v + rad_px) + 1)
    x0, x1 = max(0, int(u - rad_px)), min(w, int(u + rad_px) + 1)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    patch = depth[y0:y1, x0:x1].astype(np.float64)
    inside = (np.hypot(xx - u, yy - v) <= rad_px) & (patch > 0)
    # выкинуть точки, ушедшие от центральной глубины больше чем на 3 см:
    # это другой объект или край, а не наша площадка
    inside &= np.abs(patch - z0) < 30.0
    if inside.sum() < MIN_PATCH_PX:
        return {"ошибка": f"в пятне чашки всего {int(inside.sum())} валидных точек"}

    zs = patch[inside] / 1000.0
    pts = np.stack([(xx[inside] - K[0, 2]) * zs / K[0, 0],
                    (yy[inside] - K[1, 2]) * zs / K[1, 1],
                    zs], axis=1)

    centre = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centre)
    n = vt[2]
    if n[2] > 0:              # нормаль всегда навстречу камере
        n = -n
    res = np.abs((pts - centre) @ n) * 1000.0
    return {
        "точка_камера": centre,
        "нормаль_камера": n,
        "точек": int(inside.sum()),
        "радиус_px": rad_px,
        "остаток_мм": float(np.median(res)),
        "остаток_p95_мм": float(np.percentile(res, 95)),
        "глубина_мм": z0,
    }


def rpy_from_R(R):
    """Углы Добота (rx, ry, rz) из матрицы. Соглашение R = Rz(rz)Ry(ry)Rx(rx)."""
    ry = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    rz = np.arctan2(R[1, 0], R[0, 0])
    rx = np.arctan2(R[2, 1], R[2, 2])
    return [float(np.degrees(a)) for a in (rx, ry, rz)]


def align_to(R_now, z_target):
    """Минимальный поворот текущей ориентации так, чтобы ось Z легла на z_target.

    Минимальный — чтобы кисть не наматывала лишние обороты: у неё уже
    сработал предел скорости однажды.
    """
    z_now = R_now @ np.array([0.0, 0.0, 1.0])
    z_target = z_target / np.linalg.norm(z_target)
    axis = np.cross(z_now, z_target)
    s, c = np.linalg.norm(axis), float(np.dot(z_now, z_target))
    if s < 1e-9:
        return R_now if c > 0 else -R_now
    axis = axis / s * np.arctan2(s, c)
    return cv2.Rodrigues(axis)[0] @ R_now


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--approach", action="store_true",
                    help="подводить руку к точке. По умолчанию только считать")
    ap.add_argument("--standoff", type=float, default=120.0,
                    help="на сколько мм зависнуть НАД поверхностью вдоль нормали")
    ap.add_argument("--speed", type=int, default=10, help="скорость, %%")
    ap.add_argument("--z-floor", type=float, default=20.0,
                    help="ниже этой высоты фланец не опускать, мм")
    ap.add_argument("--z-ceiling", type=float, default=350.0,
                    help="выше этой высоты не поднимать — там камера, мм")
    ap.add_argument("--pick", action="store_true",
                    help="полный цикл: подвод, вакуум, посадка, подъём. "
                         "Включает --approach")
    ap.add_argument("--press", type=float, default=5.0,
                    help="на сколько мм продавить чашку НИЖЕ поверхности при "
                         "посадке. Резина уплотняется сжатием, без этого "
                         "чашка касается кромкой и не держит")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="сколько секунд ждать уплотнения после включения "
                         "вакуума. ES01 даёт 2.0 л/мин, фронт медленный")
    ap.add_argument("--lift", type=float, default=200.0,
                    help="на сколько мм поднять после захвата")
    ap.add_argument("--transit-z", type=float, default=260.0,
                    help="высота транзита: сначала выходим сюда над точкой, "
                         "потом спускаемся по прямой. Так путь предсказуем")
    args = ap.parse_args()
    if args.pick:
        args.approach = True

    try:
        cal, Tcb = load_handeye()
    except FileNotFoundError as e:
        print(e)
        return 1
    print(f"калибровка: набор {cal['набор']}, разброс "
          f"{cal['разброс_фланец_доска_мм']:.2f} мм")

    arm = None
    if args.approach:
        from suction.arm.client import ArmClient, run_tool
        print("=" * 66)
        if args.pick:
            print("РЕЖИМ ЗАХВАТА: рука поедет к точке, ВКЛЮЧИТ ВАКУУМ,")
            print(f"сядет на поверхность с продавливанием {args.press:g} мм")
            print(f"и поднимется на {args.lift:g} мм. Скорость {args.speed}%.")
        else:
            print("РЕЖИМ ПОДВОДА: рука будет запитана и поедет к указанной точке.")
            print(f"Зависание {args.standoff:g} мм над поверхностью, скорость "
                  f"{args.speed}%.")
        print(f"Длина инструмента взята {TOOL_LENGTH_MM:g} мм — если она врёт,")
        print("врёт и высота подвода. Аварийный стоп под рукой.")
        print("=" * 66)
        if input("\nПоехали? y/n: ").strip().lower() not in ("y", "д", "да"):
            return 0
        arm = ArmClient().connect()
        arm.enable()
        arm.speed(args.speed)

    rclpy.init()
    scene = Scene()
    clicked = {}

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["uv"] = (x, y)

    try:
        scene.wait_ready()
        cv2.namedWindow(WINDOW)

        if arm is not None:
            # Движение блокирует поток на секунды. Без прокачки окна X11
            # объявляет его повисшим — то самое "force quit or wait".
            def _pump_gui():
                scene.pump()
                if scene.rgb is not None:
                    cv2.imshow(WINDOW, scene.rgb)
                cv2.waitKey(1)
            arm.on_wait = _pump_gui

        # Qt-бэкенд создаёт окно не по namedWindow и даже не сразу после
        # первого imshow — момент непредсказуем при совместной работе с rclpy.
        # Поэтому привязку мыши повторяем в цикле, пока не пройдёт.
        mouse_bound = False
        cx, cy = WORK_CENTRE_PX
        print("Ткни мышью в точку на объекте. q — выход.\n")

        last = None
        while True:
            scene.pump()
            view = scene.rgb.copy()
            cv2.circle(view, (int(cx), int(cy)), int(WORK_RADIUS_PX),
                       (90, 90, 90), 1)
            if last:
                cv2.circle(view, last["uv"], int(last["r"]), (0, 220, 0), 2)
                cv2.putText(view, last["text"], (8, view.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1,
                            cv2.LINE_AA)
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(1) & 0xFF
            if not mouse_bound:
                try:
                    cv2.setMouseCallback(WINDOW, on_mouse)
                    mouse_bound = True
                except cv2.error:
                    pass
            if key in (ord("q"), 27):
                break

            if "uv" not in clicked:
                continue
            u, v = clicked.pop("uv")

            if np.hypot(u - cx, v - cy) > WORK_RADIUS_PX:
                print(f"({u},{v}) вне рабочего круга r<{WORK_RADIUS_PX:.0f} px — "
                      f"там карта глубины выгнута, наклон будет измерен неверно")
                continue

            est = estimate_normal(scene.depth, scene.K, u, v)
            if "ошибка" in est:
                print(f"({u},{v}): {est['ошибка']}")
                continue

            p_base = Tcb[:3, :3] @ est["точка_камера"] + Tcb[:3, 3]
            n_base = Tcb[:3, :3] @ est["нормаль_камера"]
            n_base = n_base / np.linalg.norm(n_base)
            if n_base[2] < 0:            # наружу от стола, вверх
                n_base = -n_base
            tilt = np.degrees(np.arccos(np.clip(n_base[2], -1, 1)))

            flat = est["остаток_мм"] <= MAX_RESIDUAL_MM
            print(f"\nточка ({u},{v}), глубина {est['глубина_мм']:.0f} мм, "
                  f"пятно {est['точек']} точек радиусом {est['радиус_px']:.0f} px")
            print(f"  плоскость: остаток median {est['остаток_мм']:.2f} мм, "
                  f"p95 {est['остаток_p95_мм']:.2f}"
                  + ("" if flat else "   <- НЕ ПЛОСКО, это край или дыра"))
            print(f"  в базе робота: x={p_base[0]*1000:+7.1f} "
                  f"y={p_base[1]*1000:+7.1f} z={p_base[2]*1000:+7.1f} мм")
            print(f"  нормаль:       [{n_base[0]:+.3f} {n_base[1]:+.3f} "
                  f"{n_base[2]:+.3f}], наклон от вертикали {tilt:.1f}°")

            last = {"uv": (u, v), "r": est["радиус_px"],
                    "text": f"z={est['глубина_мм']:.0f}mm  naklon={tilt:.1f}deg  "
                            f"ost={est['остаток_мм']:.1f}mm"}

            if arm is None:
                continue
            if not flat:
                print("  подвод отменён: площадка не плоская")
                continue

            R_now = np.array(cv2.Rodrigues(np.zeros(3))[0])
            from tools.calib.solve import euler_to_R
            pose_now = arm.pose()
            R_now = euler_to_R(pose_now[3:], ("z", "y", "x"), True)
            R_tgt = align_to(R_now, -n_base)          # инструмент против нормали

            # Чашка должна встать в standoff над точкой, фланец — дальше на
            # длину инструмента вдоль той же нормали. Боковое смещение чашки
            # от оси фланца вычитается в системе ЦЕЛЕВОЙ ориентации: оно
            # поворачивается вместе с рукой.
            tip = p_base * 1000.0 + n_base * args.standoff
            flange = tip + n_base * TOOL_LENGTH_MM
            ox, oy = TOOL_OFFSET_MM
            if ox or oy:
                flange = flange - R_tgt @ np.array([ox, oy, 0.0])
            rx, ry, rz = rpy_from_R(R_tgt)

            if arm is not None and not arm.reachable(
                    [flange[0], flange[1], flange[2], rx, ry, rz]):
                print("  РУКА НЕ ДОСТАНЕТ до этой точки.")
                print("  Обратная кинематика решения не имеет — цель вне")
                print("  рабочей области. Ткни ближе к основанию робота.")
                continue

            if not (args.z_floor <= flange[2] <= args.z_ceiling):
                print(f"  подвод отменён: фланец ушёл бы на z={flange[2]:.0f} мм, "
                      f"допустимо {args.z_floor:.0f}..{args.z_ceiling:.0f}")
                continue

            transit = max(args.transit_z, flange[2])
            transit = min(transit, args.z_ceiling)
            print(f"  цель фланца: x={flange[0]:+.1f} y={flange[1]:+.1f} "
                  f"z={flange[2]:+.1f}  rx={rx:+.1f} ry={ry:+.1f} rz={rz:+.1f}")
            print(f"  путь: сперва на транзитную z={transit:.0f} над точкой,"
                  f" затем спуск по прямой")
            if input("  ехать? y/n: ").strip().lower() not in ("y", "д", "да"):
                continue

            # Два этапа, чтобы путь был предсказуем: MovJ выводит руку в
            # нужную ориентацию высоко над точкой, MovL опускает по прямой.
            # Одним движением контроллер повёл бы по своей траектории, а под
            # рукой стол и объекты.
            try:
                print("  этап 1: выход над точкой...")
                arm.move_pose([flange[0], flange[1], transit, rx, ry, rz])
                if transit - flange[2] > 1.0:
                    print("  этап 2: спуск...")
                    arm.move_pose([flange[0], flange[1], flange[2], rx, ry, rz],
                                  linear=True)
            except Exception as e:
                print(f"  движение не прошло: {e}")
                codes = arm.errors()
                print(f"  коды: {', '.join(codes) if codes else 'нет'}")
                report = arm.error_report()
                if report:
                    print(f"  контроллер сообщает:\n     {report}")
                print("  сбросить: tools/arm/clear_fault.sh")
                continue

            after = arm.pose()
            print(f"  приехали: x={after[0]:+.1f} y={after[1]:+.1f} "
                  f"z={after[2]:+.1f}")
            miss = np.hypot(after[0] - flange[0], after[1] - flange[1])
            print(f"  промах по горизонтали: {miss:.1f} мм, "
                  f"по высоте {after[2]-flange[2]:+.1f} мм")

            if not args.pick:
                print(f"  ЗАЗОР от поверхности до губы чашки должен быть "
                      f"{args.standoff:g} мм.")
                print(f"  Померь линейкой. Если больше — TOOL_LENGTH_MM "
                      f"завышена, если меньше — занижена:")
                print(f"  новая = {TOOL_LENGTH_MM:g} - (зазор - {args.standoff:g})")
                continue

            # Посадка. Чашка идёт НИЖЕ поверхности на press: резина
            # уплотняется сжатием, а не касанием кромкой. Промах калибровки
            # по высоте (единицы мм) съедается той же податливостью.
            seat_tip = p_base * 1000.0 - n_base * args.press
            seat = seat_tip + n_base * TOOL_LENGTH_MM
            if ox or oy:
                seat = seat - R_tgt @ np.array([ox, oy, 0.0])
            if seat[2] < args.z_floor:
                print(f"  посадка отменена: фланец ушёл бы на z={seat[2]:.0f} мм, "
                      f"ниже порога {args.z_floor:.0f}")
                continue
            if not arm.reachable([seat[0], seat[1], seat[2], rx, ry, rz]):
                print("  до точки посадки рука не достаёт")
                continue

            # Подъём планируется ДО включения вакуума. Узнать, что рука не
            # достаёт до верхней точки, уже сидя на предмете — худший момент
            # из возможных: вакуум включён, предмет взят, а деваться некуда.
            lift_max, reason = args.lift, None
            if n_base[2] > 1e-6:
                # Потолок считается ВДОЛЬ НОРМАЛИ: проход d поднимает всего
                # d*n_base[2], и наивное d = (потолок - z) недобирает высоту
                # тем сильнее, чем сильнее наклон.
                by_ceiling = max(0.0, (args.z_ceiling - seat[2]) / n_base[2])
                if by_ceiling < lift_max:
                    lift_max = by_ceiling
                    reason = f"потолок {args.z_ceiling:.0f} мм, там камера"
            up, dist = None, lift_max
            while dist >= MIN_LIFT_MM:
                cand = seat + n_base * dist
                if arm.reachable([cand[0], cand[1], cand[2], rx, ry, rz]):
                    up = cand
                    break
                dist -= 10.0
                reason = "досягаемость руки"
            if up is None:
                print(f"  захват отменён: даже на {MIN_LIFT_MM:.0f} мм выше "
                      f"посадки рука не достаёт.")
                print("  брать предмет, который нельзя поднять, смысла нет")
                continue
            if dist < args.lift - 1.0:
                print(f"  подъём урезан с {args.lift:.0f} до {dist:.0f} мм "
                      f"({reason})")

            try:
                print("  вакуум ВКЛ")
                arm.gripper(True)
                print(f"  посадка: спуск на {args.standoff + args.press:.0f} мм "
                      f"по нормали...")
                arm.move_pose([seat[0], seat[1], seat[2], rx, ry, rz],
                              linear=True)
                print(f"  жду уплотнения {args.settle:g} с...")
                time.sleep(args.settle)

                print(f"  подъём на {np.linalg.norm(up - seat):.0f} мм...")
                # По суставам, не по прямой: у отрыва от поверхности нет
                # причины идти строго линейно, а прямая линия здесь может
                # потребовать пройти через переворот кисти (rx около 180°)
                # и получить отказ команды. MovJ сам найдёт путь по суставам.
                arm.move_pose([up[0], up[1], up[2], rx, ry, rz], linear=False)
            except Exception as e:
                print(f"  движение не прошло: {e}")
                codes = arm.errors()
                print(f"  коды: {', '.join(codes) if codes else 'нет'}")
                print("  сбросить: tools/arm/clear_fault.sh")
                print("  вакуум оставляю включённым — придержи предмет")
                continue

            print(f"  подняли. Выход ES01: "
                  f"{'включён' if arm.gripper_state() else 'ВЫКЛЮЧЕН'}")
            print("  Держит или нет — глазами: обратной связи по разрежению")
            print("  ещё нет, это и есть предмет работы.")
            if input("  отпустить? y/n: ").strip().lower() in ("y", "д", "да"):
                arm.gripper(False)
                print("  вакуум ВЫКЛ")
    finally:
        cv2.destroyAllWindows()
        for _ in range(4):
            cv2.waitKey(1)
        scene.destroy_node()
        rclpy.shutdown()
        if arm is not None:
            # Вакуум НЕ выключаем сами: если в чашке предмет, он упадёт.
            # Но и молчать нельзя — насос останется работать после выхода.
            try:
                if arm.gripper_state():
                    print("\nВАКУУМ ОСТАЛСЯ ВКЛЮЧЁН. Выключить: "
                          "tools/arm/gripper.sh off")
            except Exception:
                pass
            arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(run_tool(main))
