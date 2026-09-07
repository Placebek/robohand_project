#!/usr/bin/env python3
"""Съёмка поз для hand-eye калибровки. Ручное ведение, без слепых движений.

Схема eye-to-hand: камера неподвижна над столом, ChArUco-доска закреплена на
фланце. Робот возит доску, на каждой позе пишется кадр RGB и РЕАЛЬНЫЕ углы
суставов из потока состояния. Дальше по набору решается AX=XB.

Позы задаются рукой: скрипт включает режим ручного ведения, ты ставишь руку
куда хочешь и жмёшь Enter. Никаких запрограммированных движений вслепую.

ГЛАВНОЕ ПРО ПОЗЫ: обязательно менять ОРИЕНТАЦИЮ, а не только положение. На
чистой трансляции AX=XB вырождается и выдаёт красивую невязку при полностью
неверной матрице. Кренить доску во все стороны, разворачивать, менять высоту.

Масштаб доски берётся измеренный (measured_square_mm из charuco_board.yaml),
а не номинальный: печать дала 28.41 мм вместо 30, и на номинале трансляция
уехала бы на 5%.

Запуск:
  tools/calib/capture.sh              # 18 поз
  tools/calib/capture.sh --poses 25
"""
import argparse
import json
import sys
import time
from datetime import date
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

import cv2                      # noqa: E402
import numpy as np              # noqa: E402
import rclpy                    # noqa: E402
import yaml                     # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node     # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

from suction.arm.client import ArmClient, ArmError, run_tool  # noqa: E402
from suction.config import DATA_CALIB, RGB_TOPIC, RGB_INFO_TOPIC  # noqa: E402

MIN_CORNERS = 12        # меньше — поза бесполезна для решения
GOOD_CORNERS = 20


class Eye(Node):
    """RGB-кадры и интринсики цветной камеры."""

    def __init__(self):
        super().__init__("handeye_capture")
        self.bridge = CvBridge()
        self.frame = None
        self.K = None
        self.D = None
        self.create_subscription(Image, RGB_TOPIC, self._on_image, 10)
        self.create_subscription(CameraInfo, RGB_INFO_TOPIC, self._on_info, 10)

    def _on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(1, -1)

    def wait(self, what, getter, timeout=10.0):
        deadline = time.monotonic() + timeout
        while getter() is None:
            if time.monotonic() > deadline:
                raise RuntimeError(f"{what} не пришло за {timeout:g} с — "
                                   f"запущен ли драйвер камеры?")
            rclpy.spin_once(self, timeout_sec=0.1)

    def fresh_frame(self, settle=0.4):
        """Свежий кадр: подождать, чтобы в буфере не осталось кадра с движения."""
        self.frame = None
        end = time.monotonic() + settle
        while time.monotonic() < end or self.frame is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.frame.copy()

    def pump(self):
        """Один оборот ROS без ожидания. Для цикла превью."""
        rclpy.spin_once(self, timeout_sec=0.01)
        return self.frame


def load_board():
    """Доска по ИЗМЕРЕННОМУ масштабу печати, не по номиналу."""
    spec = yaml.safe_load((DATA_CALIB / "charuco_board.yaml").read_text())
    nominal_mm = spec["square_length_m"] * 1000.0
    measured_mm = spec.get("measured_square_mm") or nominal_mm
    k = measured_mm / nominal_mm
    square_m = spec["square_length_m"] * k
    marker_m = spec["marker_length_m"] * k
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, spec["dictionary"]))
    board = cv2.aruco.CharucoBoard_create(
        spec["squares_x"], spec["squares_y"], square_m, marker_m, dictionary)
    print(f"доска: {spec['squares_x']}x{spec['squares_y']}, клетка "
          f"{square_m*1000:.2f} мм (номинал {nominal_mm:.0f}, масштаб печати "
          f"{k:.4f}), маркер {marker_m*1000:.2f} мм")
    return board, dictionary


def board_rotation(corners, ids, board, K, D):
    """Матрица поворота доски в кадре. None, если поза не оценилась.

    Берётся именно поза доски по камере, а не углы фланца: тут нет
    неоднозначности с соглашением об углах Эйлера, вектор Родрига однозначен.
    Доска жёстко на фланце, так что разнообразие её ориентаций равно
    разнообразию ориентаций фланца.
    """
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        corners, ids, board, K, D, None, None)
    if not ok:
        return None
    return cv2.Rodrigues(rvec)[0]


def rotation_diversity(rotations):
    """Насколько богат набор ориентаций. Возвращает (макс_угол, сингулярные).

    Для каждой позы берётся поворот относительно первой, из него — ось и угол.
    Оси, взвешенные углом, складываются в матрицу 3xN. Её сингулярные числа
    показывают, в скольких независимых направлениях реально кручено:
    третье число близко к нулю — все оси лежат в одной плоскости, и вокруг
    нормали к этой плоскости R_X остаётся неопределённой.
    """
    if len(rotations) < 2:
        return 0.0, (0.0, 0.0, 0.0)
    axes, angles = [], []
    R0 = rotations[0]
    for R in rotations[1:]:
        rel = R @ R0.T
        rvec = cv2.Rodrigues(rel)[0].reshape(3)
        angle = float(np.linalg.norm(rvec))
        if angle > 1e-6:
            axes.append(rvec)            # длина вектора = угол в радианах
            angles.append(np.degrees(angle))
    if not axes:
        return 0.0, (0.0, 0.0, 0.0)
    sv = np.linalg.svd(np.array(axes).T, compute_uv=False)
    sv = tuple(float(x) for x in np.pad(sv, (0, max(0, 3 - len(sv)))))
    return max(angles), sv[:3]


def diversity_verdict(rotations):
    """Строка для человека: хватает ли разброса ориентаций."""
    max_angle, sv = rotation_diversity(rotations)
    if sv[0] < 1e-9:
        return "ориентация не менялась вообще — решение выродится", False
    ratio = sv[2] / sv[0]
    text = (f"разброс ориентаций: макс. поворот {max_angle:.0f}°, "
            f"оси {sv[0]:.2f}/{sv[1]:.2f}/{sv[2]:.2f}, "
            f"третья/первая = {ratio:.3f}")
    if max_angle < 20:
        return text + "\n   МАЛО: повороты меньше 20°, кренить сильнее", False
    if ratio < 0.15:
        return (text + "\n   ПЛОСКО: оси вращения почти в одной плоскости. "
                "Кренить в ДРУГУЮ сторону", False)
    return text + "\n   разброс достаточный", True


def detect(frame, board, dictionary):
    """Углы ChArUco на кадре. Возвращает (corners, ids) или (None, None)."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) == 0:
        return None, None
    n, corners, ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board)
    if n is None or n < 4:
        return None, None
    return corners, ids


WINDOW = "hand-eye: ПРОБЕЛ снять, s пропустить, q закончить"


def draw_overlay(frame, corners, ids, shot, total, verdict):
    """Кадр с разметкой: найденные углы и подсказки."""
    view = frame.copy()
    n = 0 if ids is None else len(ids)
    if n:
        cv2.aruco.drawDetectedCornersCharuco(view, corners, ids, (0, 255, 0))

    if n >= GOOD_CORNERS:
        color, state = (0, 220, 0), "готов снимать"
    elif n >= MIN_CORNERS:
        color, state = (0, 200, 220), "мало углов, но сойдёт"
    else:
        color, state = (0, 0, 255), f"НЕ СНЯТЬ: нужно {MIN_CORNERS}+"

    bar = view.shape[0] - 52
    cv2.rectangle(view, (0, bar), (view.shape[1], view.shape[0]), (0, 0, 0), -1)
    cv2.putText(view, f"uglov: {n}  -  {state}", (8, bar + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    cv2.putText(view, f"snyato {shot}/{total}   {verdict}", (8, bar + 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return view


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        print("\nНужен интерактивный терминал.")
        sys.exit(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", type=int, default=18, help="сколько поз снять")
    ap.add_argument("--sensitivity", type=int, default=None,
                    help="податливость ручного ведения 1-90, меньше — туже. "
                         "E6 эту команду отвергает, по умолчанию не трогаем")
    ap.add_argument("--hold-board", action="store_true",
                    help="включить присоску на старте: доска на лёгкой жёсткой "
                         "подложке держится самим захватом. Выключать вакуум "
                         "скрипт НЕ будет — иначе доска упадёт в конце серии")
    ap.add_argument("--no-drag", action="store_true",
                    help="не включать ручное ведение: руку ставишь пультом "
                         "или кнопкой на фланце, скрипт только пишет позы")
    ap.add_argument("--out", default=None, help="куда писать (по умолчанию по дате)")
    args = ap.parse_args()

    out = Path(args.out) if args.out else DATA_CALIB / f"handeye_{date.today():%Y%m%d}"
    out.mkdir(parents=True, exist_ok=True)

    board, dictionary = load_board()

    rclpy.init()
    eye = Eye()
    try:
        eye.wait("интринсики RGB", lambda: eye.K)
        eye.wait("кадр RGB", lambda: eye.frame)
        print(f"камера: fx={eye.K[0,0]:.1f} fy={eye.K[1,1]:.1f} "
              f"cx={eye.K[0,2]:.1f} cy={eye.K[1,2]:.1f}")

        print()
        print("=" * 66)
        print("СЪЁМКА ПОЗ ДЛЯ HAND-EYE")
        print("=" * 66)
        print("  Сейчас рука будет ЗАПИТАНА и переведена в ручное ведение —")
        print("  она обмякнет и просядет под своим весом. ДЕРЖИ ЕЁ РУКОЙ,")
        print("  прежде чем отвечать. Аварийный стоп под другой рукой.")
        print()
        print("  Доска должна быть ЖЁСТКО закреплена на фланце. Если она")
        print("  сдвинется посреди серии — весь набор поз в мусор.")
        if args.hold_board:
            print()
            print("  --hold-board: доску держит присоска. Вакуум включится")
            print("  сейчас и НЕ выключится в конце — иначе доска упадёт.")
            print("  Держание 55 кПа на чашке 35 мм это ~53 Н, для лёгкой")
            print("  подложки запас стократный. Но подложка должна быть")
            print("  ЖЁСТКОЙ и ПЛОСКОЙ: на мятой бумаге чашка не уплотнится.")
        print()
        print(f"  Снимаем {args.poses} поз. На каждой МЕНЯЙ ОРИЕНТАЦИЮ доски,")
        print("  не только положение: наклоны во все стороны, развороты,")
        print("  разная высота. Держи доску на 250-350 мм от камеры — там")
        print("  маркеры крупные. Без разброса ориентаций решение вырождается.")
        print("=" * 66)
        if not ask("\nДержу руку, стоп под рукой. Поехали? y/n: ").startswith(("y", "д")):
            return 0

        records = []
        rotations = []
        with ArmClient() as arm:
            print(f"\nрука: режим {arm.mode()} — {arm.mode_name()}")
            if args.hold_board:
                arm.gripper(True)
                time.sleep(1.0)
                print("вакуум включён — проверь, что доска держится крепко,")
                print("подёргай её рукой, прежде чем идти дальше")
                if not ask("держится? y/n: ").startswith(("y", "д")):
                    print("Останавливаюсь. Вакуум оставляю включённым.")
                    return 1
            arm.enable()
            dragging = False
            if args.no_drag:
                print("запитана. ручное ведение не трогаю (--no-drag).")
                print("ставь руку как умеешь — пультом, кнопкой ведения на")
                print("фланце, из DobotStudio. Скрипт просто пишет, где она.\n")
            else:
                print("запитана. включаю ручное ведение...")
                try:
                    arm.drag(True, sensitivity=args.sensitivity)
                    dragging = True
                    print("рука податлива — води её руками\n")
                except ArmError as e:
                    print(f"  StartDrag не прошёл: {e}")
                    print("  Значит води руку иначе: кнопкой ручного ведения на")
                    print("  фланце, пультом или из DobotStudio. Съёмка от этого")
                    print("  не зависит — скрипт читает, где рука оказалась.\n")

            print("Окно превью: зелёным подсвечены найденные углы доски.")
            print("ПРОБЕЛ — снять позу, s — пропустить, q — закончить.")
            print("Клавиши работают В ОКНЕ, не в терминале.\n")
            cv2.namedWindow(WINDOW)

            try:
                i = 0
                verdict_line = ""
                while i < args.poses:
                    eye.pump()
                    if eye.frame is None:
                        continue
                    frame = eye.frame.copy()
                    corners, ids = detect(frame, board, dictionary)
                    n = 0 if ids is None else len(ids)
                    cv2.imshow(WINDOW, draw_overlay(frame, corners, ids,
                                                    i, args.poses, verdict_line))

                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("s"):
                        print(f"поза {i+1}: пропущена")
                        continue
                    if key != 32:            # пробел
                        continue

                    if n < MIN_CORNERS:
                        print(f"поза {i+1}: углов {n}, нужно {MIN_CORNERS}+ — "
                              f"не записана. Доска не в кадре, косо или далеко")
                        continue

                    R = board_rotation(corners, ids, board, eye.K, eye.D)
                    if R is None:
                        print(f"поза {i+1}: поза доски не оценилась, переставь")
                        continue

                    joints = arm.joints()
                    pose = arm.pose()
                    name = f"pose_{i:02d}"
                    cv2.imwrite(str(out / f"{name}.png"), frame)
                    records.append({
                        "имя": name,
                        "углы_суставов": joints,
                        "поза_фланца": pose,
                        "углов_доски": int(n),
                        "поворот_доски": R.tolist(),
                        "corners": corners.reshape(-1, 2).tolist(),
                        "ids": ids.reshape(-1).tolist(),
                    })
                    rotations.append(R)
                    mark = "хорошо" if n >= GOOD_CORNERS else "маловато"
                    print(f"поза {i+1}/{args.poses} записана: углов {n} ({mark}), "
                          f"фланец z={pose[2]:.0f} мм")
                    if len(rotations) >= 3:
                        text, _ = diversity_verdict(rotations)
                        verdict_line = text.splitlines()[-1].strip()
                        print(f"   {text}")
                    i += 1
            finally:
                cv2.destroyAllWindows()
                for _ in range(4):
                    cv2.waitKey(1)
                if dragging:
                    print("\nвыключаю ручное ведение — ДЕРЖИ РУКУ")
                    try:
                        arm.drag(False)
                    except Exception as e:
                        print(f"  StopDrag не прошёл: {e}")

        meta = {
            "снято": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "поз": len(records),
            "K": eye.K.tolist(),
            "D": eye.D.tolist() if eye.D is not None else None,
            "позы": records,
        }
        (out / "poses.json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=1))
        print(f"\nсохранено {len(records)} поз в {out}")

        text, ok = diversity_verdict(rotations)
        print(text)
        if args.hold_board:
            print("\nВакуум остался ВКЛЮЧЁННЫМ — доска всё ещё на присоске.")
            print("Придержи доску и выключи:  tools/arm/gripper.sh off")

        if len(records) < 10:
            print("\nМеньше десяти поз — решение будет неустойчивым, досними.")
        elif not ok:
            print("\nНабор снят, но вращательная часть AX=XB на нём определится")
            print("плохо. Досними поз с кренами вокруг других осей — иначе")
            print("получишь красивую невязку при неверной матрице.")
        else:
            print("\nНабор годный. Дальше: решение AX=XB.")
    finally:
        eye.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(run_tool(main))
