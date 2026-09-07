#!/usr/bin/env python3
"""Решение AX=XB по снятому набору поз. Даёт матрицу камера -> база робота.

Схема eye-to-hand: камера неподвижна над столом, доска на фланце. OpenCV
решает обратную задачу (камера на руке, доска неподвижна), поэтому позы
фланца подаются ОБРАЩЁННЫМИ — тогда на выходе получается ровно нужная
матрица «камера -> база».

Две вещи, которые здесь легко сделать молча неправильно:

1. Соглашение об углах Эйлера. Контроллер отдаёт ориентацию фланца тройкой
   rx, ry, rz, но порядок осей и способ composition в документации не
   зафиксированы. Ошибка в порядке даёт решение, которое сходится и выглядит
   правдоподобно. Поэтому перебираются все двенадцать вариантов, и выбор
   делается по проверке ниже, а не по вере.

2. Проверка. Доска жёстко на фланце, значит преобразование «фланец -> доска»
   ОДНО И ТО ЖЕ на всех позах. Считаем его для каждой позы и смотрим разброс.
   Верное решение даёт разброс в доли миллиметра и доли градуса; неверное —
   сантиметры и десятки градусов. Это независимая проверка: она не участвует
   в решении, она его судит.

Запуск:
  tools/calib/solve.sh
  tools/calib/solve.sh --data data/calib/handeye_20260903
"""
import argparse
import itertools
import json
import sys
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

import cv2          # noqa: E402
import numpy as np  # noqa: E402
import yaml         # noqa: E402

from suction.config import DATA_CALIB  # noqa: E402

METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def rot(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_variants():
    """Двенадцать способов собрать матрицу из тройки (rx, ry, rz)."""
    out = []
    for order in itertools.permutations("xyz"):
        for reverse in (False, True):
            out.append((order, reverse))
    return out


def euler_to_R(rxyz, order, reverse):
    angles = list(rxyz)[::-1] if reverse else list(rxyz)
    R = np.eye(3)
    for axis, ang in zip(order, angles):
        R = R @ rot(axis, ang)
    return R


def T(R, t):
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = np.asarray(t).reshape(3)
    return m


def load_board():
    spec = yaml.safe_load((DATA_CALIB / "charuco_board.yaml").read_text())
    nominal = spec["square_length_m"] * 1000.0
    k = (spec.get("measured_square_mm") or nominal) / nominal
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, spec["dictionary"]))
    board = cv2.aruco.CharucoBoard_create(
        spec["squares_x"], spec["squares_y"],
        spec["square_length_m"] * k, spec["marker_length_m"] * k, dictionary)
    return board


def board_poses(data, board):
    """Позы доски в кадре камеры: (R, t в метрах) для каждой записи."""
    K = np.array(data["K"], dtype=np.float64)
    D = np.array(data["D"], dtype=np.float64) if data.get("D") else None
    out = []
    for rec in data["позы"]:
        corners = np.array(rec["corners"], dtype=np.float32).reshape(-1, 1, 2)
        ids = np.array(rec["ids"], dtype=np.int32).reshape(-1, 1)
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            corners, ids, board, K, D, None, None)
        if not ok:
            out.append(None)
            continue
        out.append((cv2.Rodrigues(rvec)[0], tvec.reshape(3)))
    return out


def consistency(T_cam2base, flange_T, board_T):
    """Разброс преобразования «фланец -> доска». Меньше — вернее решение.

    Доска приклеена к фланцу, значит это преобразование постоянно. Считаем
    его на каждой позе и меряем, насколько позы расходятся между собой.
    """
    mats = []
    for Tg, bp in zip(flange_T, board_T):
        if bp is None:
            continue
        Tb = T(bp[0], bp[1])          # поза доски приходит парой (R, t)
        mats.append(np.linalg.inv(Tg) @ T_cam2base @ Tb)
    if len(mats) < 2:
        return np.inf, np.inf
    trans = np.array([m[:3, 3] for m in mats]) * 1000.0     # мм
    spread_mm = float(np.linalg.norm(trans.std(axis=0)))
    ref = mats[0][:3, :3]
    angles = []
    for m in mats[1:]:
        rel = m[:3, :3] @ ref.T
        angles.append(np.degrees(np.linalg.norm(cv2.Rodrigues(rel)[0])))
    return spread_mm, float(np.std(angles) + np.mean(angles)) if angles else 0.0


def compliance_check(T_cam2base, flange_T, board_T, flange_raw):
    """Не пружинит ли крепление доски.

    Если доску держит присоска, резина чашки деформируется под моментом от
    веса подложки, и «фланец -> доска» перестаёт быть постоянным: он поедет
    тем сильнее, чем круче накренена рука. Высокая корреляция между наклоном
    от вертикали и отклонением — приговор креплению, а не калибровке.
    """
    tilts, dev_deg, dev_mm = [], [], []
    ref = None
    for Tg, bp, raw in zip(flange_T, board_T, flange_raw):
        if bp is None:
            continue
        M = np.linalg.inv(Tg) @ T_cam2base @ T(bp[0], bp[1])
        if ref is None:
            ref = M
        axis = Tg[:3, :3] @ np.array([0.0, 0.0, 1.0])
        tilts.append(np.degrees(np.arccos(abs(np.clip(axis[2], -1, 1)))))
        rel = M[:3, :3] @ ref[:3, :3].T
        dev_deg.append(np.degrees(np.linalg.norm(cv2.Rodrigues(rel)[0])))
        dev_mm.append(np.linalg.norm(M[:3, 3] - ref[:3, 3]) * 1000.0)
    if len(tilts) < 4:
        return None
    t = np.array(tilts)
    return {
        "наклон_мин": float(t.min()), "наклон_макс": float(t.max()),
        "корр_угол": float(np.corrcoef(t, dev_deg)[0, 1]),
        "корр_смещение": float(np.corrcoef(t, dev_mm)[0, 1]),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=None, help="папка со снятыми позами")
    ap.add_argument("--top", type=int, default=5, help="сколько вариантов показать")
    args = ap.parse_args()

    folder = Path(args.data) if args.data else max(
        DATA_CALIB.glob("handeye_*"), key=lambda p: p.name)
    data = json.loads((folder / "poses.json").read_text())
    print(f"набор: {folder}   поз: {data['поз']}")

    board = load_board()
    b_poses = board_poses(data, board)
    good = sum(1 for x in b_poses if x is not None)
    print(f"поза доски оценилась на {good} из {len(b_poses)} кадров")
    if good < 6:
        print("Слишком мало — решать нечего.")
        return 1

    flange_raw = [rec["поза_фланца"] for rec in data["позы"]]

    results = []
    for order, reverse in euler_variants():
        flange_T = [T(euler_to_R(p[3:], order, reverse), np.array(p[:3]) / 1000.0)
                    for p in flange_raw]

        # eye-to-hand: подаём ОБРАЩЁННЫЕ позы фланца
        R_b2g, t_b2g, R_t2c, t_t2c = [], [], [], []
        for Tg, bp in zip(flange_T, b_poses):
            if bp is None:
                continue
            inv = np.linalg.inv(Tg)
            R_b2g.append(inv[:3, :3])
            t_b2g.append(inv[:3, 3])
            R_t2c.append(bp[0])
            t_t2c.append(bp[1])

        for mname, mid in METHODS.items():
            try:
                R_c2b, t_c2b = cv2.calibrateHandEye(
                    R_b2g, t_b2g, R_t2c, t_t2c, method=mid)
            except cv2.error:
                continue
            if not np.all(np.isfinite(R_c2b)) or not np.all(np.isfinite(t_c2b)):
                continue
            Tcb = T(R_c2b, np.asarray(t_c2b).reshape(3))
            mm, deg = consistency(Tcb, flange_T, b_poses)
            results.append({
                "порядок": "".join(order) + ("-обр" if reverse else ""),
                "метод": mname, "разброс_мм": mm, "разброс_град": deg,
                "T": Tcb,
                "_euler": (order, reverse),      # чтобы не разбирать строку назад
                "_flange_T": flange_T,
            })

    results.sort(key=lambda r: r["разброс_мм"] + 10 * r["разброс_град"])
    print(f"\nперебрано вариантов: {len(results)}")
    print(f"\n{'порядок':>10} {'метод':>11} {'разброс, мм':>12} {'разброс, °':>11}")
    print("-" * 48)
    for r in results[:args.top]:
        print(f"{r['порядок']:>10} {r['метод']:>11} "
              f"{r['разброс_мм']:>12.2f} {r['разброс_град']:>11.2f}")

    best = results[0]
    print(f"\nлучший: порядок {best['порядок']}, метод {best['метод']}")
    print(f"разброс «фланец -> доска»: {best['разброс_мм']:.2f} мм, "
          f"{best['разброс_град']:.2f}°")

    comp = compliance_check(best["T"], best["_flange_T"], b_poses, flange_raw)

    ok = best["разброс_мм"] <= 5 and best["разброс_град"] <= 2
    if comp:
        print(f"\nнаклон фланца от вертикали: "
              f"{comp['наклон_мин']:.1f}..{comp['наклон_макс']:.1f}°")
        print(f"корреляция наклон <-> угловое отклонение:  "
              f"{comp['корр_угол']:+.3f}")
        print(f"корреляция наклон <-> смещение:            "
              f"{comp['корр_смещение']:+.3f}")

    if not ok:
        print("\nРАЗБРОС ВЕЛИК — решение неточно.")
        if comp and comp["корр_угол"] > 0.6:
            print("И причина видна: отклонение растёт вместе с наклоном руки.")
            print("Это ПРУЖИНИТ КРЕПЛЕНИЕ, а не врёт калибровка. Резиновая")
            print("чашка деформируется под моментом от веса подложки.")
            print("Лечится жёсткостью, а не пересъёмкой тем же способом:")
            print("  - подложку меньше и легче, момент падает пропорционально")
            print("  - чашку строго в центр тяжести подложки")
            print("  - лучше всего — механический упор, а не только вакуум")
        else:
            print("Причины: доска сдвинулась, подложка гнётся, мало ориентаций.")
    else:
        print("\nРазброс мал — решение согласовано на всех позах.")
        if comp and comp["корр_угол"] > 0.6:
            print("Но отклонение всё же растёт с наклоном — крепление пружинит,")
            print("просто пока в пределах допуска.")

    Tcb = best["T"]
    print(f"\nматрица камера -> база (трансляция в мм):")
    for row in range(3):
        r = Tcb[row]
        print(f"  [{r[0]:+.5f} {r[1]:+.5f} {r[2]:+.5f}   {r[3]*1000:+9.2f}]")
    print(f"\nкамера стоит в базовых координатах: "
          f"x={Tcb[0,3]*1000:+.1f} y={Tcb[1,3]*1000:+.1f} z={Tcb[2,3]*1000:+.1f} мм")

    out = folder.parent / "handeye.json"

    # Рядом с рабочим файлом кладём копию, привязанную к набору: решение по
    # другому набору иначе затирает предыдущее молча, и вернуться не к чему.
    if out.exists():
        try:
            prev = json.loads(out.read_text())
        except (OSError, ValueError):
            prev = {}
        if prev.get("набор") and prev["набор"] != folder.name:
            print(f"\nВНИМАНИЕ: заменяю решение по набору {prev['набор']} "
                  f"(разброс {prev.get('разброс_фланец_доска_мм', 0):.2f} мм) "
                  f"решением по {folder.name}.")
            print("Прежнее остаётся в "
                  f"{prev['набор']}.json, если камеру не двигали —")
            print("сверь отпечаток зоны: tools/camera/zone_check.sh")

    payload = json.dumps({
        "набор": folder.name,
        "поз": good,
        "порядок_углов_эйлера": best["порядок"],
        "метод": best["метод"],
        "разброс_фланец_доска_мм": best["разброс_мм"],
        "разброс_фланец_доска_град": best["разброс_град"],
        "податливость_крепления": comp,
        "T_камера_в_базу": Tcb.tolist(),
        "комментарий": "трансляция в метрах; позы фланца подавались обращёнными",
    }, ensure_ascii=False, indent=2)
    out.write_text(payload)
    (folder.parent / f"{folder.name}.json").write_text(payload)
    print(f"\nсохранено: {out}")
    print(f"копия набора:  {folder.parent / (folder.name + '.json')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
