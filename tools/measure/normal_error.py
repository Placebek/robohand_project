#!/usr/bin/env python3
"""Ошибка оценки нормали по всему кадру. Главный измеритель RQ2.

Под камеру кладётся ПЛОСКАЯ поверхность (пустой стол годится). Раз она
плоская, истинный наклон нормали известен и равен нулю — значит всё, что
скрипт намеряет, это ошибка метода. Промеряется сетка точек, для каждой
берётся окрестность размером с чашку и подгоняется плоскость.

Считается три вещи:
  - ошибка нормали по кольцам радиуса: где кадр годен, а где нет;
  - выгнутость карты глубины: отклонение от плоскости, подогнанной по центру;
  - предсказывает ли остаток от подгонки ошибку — то есть можно ли по нему
    отбраковывать ненадёжные точки на лету.

Запуск:
  tools/measure/normal_error.sh
  tools/measure/normal_error.sh --step 8 --max-radius 320
"""
import argparse
import json
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import rclpy        # noqa: E402

from suction.config import (DATA_CALIB, WORK_CENTRE_PX,  # noqa: E402
                            WORK_RADIUS_PX)
from tools.measure.pick_normal import Scene, estimate_normal  # noqa: E402

RINGS = ((0, 60), (60, 120), (120, 180), (180, 240), (240, 300), (300, 400))


def collect_depth(scene, seconds=8.0, frames=50):
    got = []
    end = time.monotonic() + seconds
    while time.monotonic() < end and len(got) < frames:
        scene.pump()
        if scene.depth is not None:
            got.append(scene.depth.copy())
            scene.depth = None
    if not got:
        raise RuntimeError("кадров глубины нет")
    st = np.stack([f.astype(np.float32) for f in got])
    st[st == 0] = np.nan
    return np.nanmedian(st, axis=0), len(got)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=int, default=12, help="шаг сетки в пикселях")
    ap.add_argument("--max-radius", type=float, default=300.0,
                    help="докуда мерить, px")
    ap.add_argument("--save", action="store_true", help="записать результат в data/calib")
    args = ap.parse_args()

    cal = DATA_CALIB / "handeye.json"
    Tcb = np.array(json.loads(cal.read_text())["T_камера_в_базу"]) if cal.exists() else None
    if Tcb is None:
        print("нет handeye.json — наклон будет считаться в системе камеры")

    cx, cy = WORK_CENTRE_PX
    rclpy.init()
    scene = Scene()
    try:
        scene.wait_ready()
        depth, nframes = collect_depth(scene)
        print(f"кадров усреднено: {nframes}")
        print("ПОВЕРХНОСТЬ ДОЛЖНА БЫТЬ ПЛОСКОЙ — иначе меряется она, а не метод.\n")

        # --- ошибка нормали ------------------------------------------
        rows = []
        h, w = depth.shape
        for v in range(20, h - 20, args.step):
            for u in range(20, w - 20, args.step):
                r = float(np.hypot(u - cx, v - cy))
                if r > args.max_radius:
                    continue
                e = estimate_normal(np.nan_to_num(depth), scene.K, u, v)
                if "ошибка" in e:
                    continue
                n = e["нормаль_камера"]
                if Tcb is not None:
                    n = Tcb[:3, :3] @ n
                n = n / np.linalg.norm(n)
                if n[2] < 0:
                    n = -n
                rows.append((r, float(np.degrees(np.arccos(np.clip(n[2], -1, 1)))),
                             e["остаток_мм"]))
        a = np.array(rows)
        print(f"точек промерено: {len(a)}\n")
        print(f"{'радиус, px':>12} {'точек':>7} {'ошибка med':>11} {'p95':>8} "
              f"{'остаток med':>12}")
        for lo, hi in RINGS:
            m = (a[:, 0] >= lo) & (a[:, 0] < hi)
            if m.sum() < 5:
                continue
            mark = "" if hi <= WORK_RADIUS_PX else "   вне круга"
            print(f"{lo:>5}-{hi:<6} {int(m.sum()):>7} {np.median(a[m,1]):>10.2f}° "
                  f"{np.percentile(a[m,1],95):>7.2f}° {np.median(a[m,2]):>11.2f}{mark}")

        inner = a[a[:, 0] < WORK_RADIUS_PX]
        print(f"\nв круге r<{WORK_RADIUS_PX:.0f}: {len(inner)} точек, "
              f"ошибка median {np.median(inner[:,1]):.2f}°, "
              f"p95 {np.percentile(inner[:,1],95):.2f}°")
        print(f"  за порогом 5°: {100*(inner[:,1]>5).mean():.1f}% точек")

        # --- выгнутость карты ----------------------------------------
        yy, xx = np.mgrid[0:h, 0:w]
        r = np.hypot(xx - cx, yy - cy)
        m = np.isfinite(depth) & (r < 120)
        A = np.c_[xx[m], yy[m], np.ones(m.sum())]
        coef, *_ = np.linalg.lstsq(A, depth[m], rcond=None)
        dev = depth - (coef[0] * xx + coef[1] * yy + coef[2])
        print(f"\nвыгнутость карты (плоскость подогнана по r<120):")
        for lo, hi in RINGS:
            k = np.isfinite(dev) & (r >= lo) & (r < hi)
            if k.sum() < 50:
                continue
            print(f"{lo:>5}-{hi:<6} {np.nanmedian(dev[k]):>+8.2f} мм")

        # --- годится ли остаток как признак отбраковки ----------------
        c = float(np.corrcoef(inner[:, 2], inner[:, 1])[0, 1])
        print(f"\nкорреляция «остаток <-> ошибка нормали»: {c:+.3f}")
        for thr in (0.3, 0.5, 0.8):
            k = inner[inner[:, 2] <= thr]
            if len(k) < 10:
                continue
            print(f"  отбор по остатку <= {thr} мм оставляет "
                  f"{100*len(k)/len(inner):>4.0f}% точек: "
                  f"ошибка med {np.median(k[:,1]):.2f}°, "
                  f"p95 {np.percentile(k[:,1],95):.2f}°")

        if args.save:
            out = DATA_CALIB / f"normal_error_{time.strftime('%Y%m%d_%H%M')}.json"
            out.write_text(json.dumps({
                "точек": len(a),
                "ошибка_med_в_круге": float(np.median(inner[:, 1])),
                "ошибка_p95_в_круге": float(np.percentile(inner[:, 1], 95)),
                "доля_за_порогом_5": float((inner[:, 1] > 5).mean()),
                "корреляция_остаток_ошибка": c,
                "по_кольцам": [[lo, hi,
                                float(np.median(a[(a[:,0]>=lo)&(a[:,0]<hi), 1]))]
                               for lo, hi in RINGS
                               if ((a[:,0]>=lo)&(a[:,0]<hi)).sum() >= 5],
            }, ensure_ascii=False, indent=2))
            print(f"\nсохранено: {out}")
    finally:
        scene.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
