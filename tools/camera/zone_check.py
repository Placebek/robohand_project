#!/usr/bin/env python3
"""Что реально попадает в рабочий круг на текущей высоте подвеса.

Рабочая зона ограничена не кадром, а кругом r < 180 px вокруг главной точки
(320, 275): дальше карта глубины радиально выгибается и нормали становятся
мусорными. Скрипт берёт 40 кадров, медианит их и отвечает на три вопроса:
есть ли дыры внутри круга, ложится ли всё на одну плоскость, и не залезли ли
в круг край стола, пол или база робота.

Гонять после КАЖДОГО перемещения камеры и обязательно после её закрепления —
последний прогон перед hand-eye.

Коды возврата: 0 всё хорошо, 1 нет кадров, 2 камеру сдвинули относительно
эталона. Последнее важно: сдвиг после калибровки не проявляется ничем — матрица
на месте, точки переводятся, рука едет мимо.

Запуск:
  tools/camera/zone_check.sh
  tools/camera/zone_check.sh --save     записать эталон плоскости
  tools/camera/zone_check.sh --quiet    только вердикт, для проверок из обёрток
"""
import sys, time
from pathlib import Path
ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))
import warnings
import numpy as np, rclpy
warnings.filterwarnings('ignore', 'All-NaN slice encountered')
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from suction.config import DEPTH_TOPIC, DEPTH_INFO_TOPIC

CX, CY, R = 320.0, 275.0, 180.0     # главная точка и рабочий радиус

QUIET = "--quiet" in sys.argv


def say(*args, **kw):
    """Подробности. В тихом режиме печатается только вердикт по сдвигу."""
    if not QUIET:
        print(*args, **kw)

class T(Node):
    def __init__(self):
        super().__init__("zone"); self.b=CvBridge(); self.frames=[]; self.k=None
        self.create_subscription(Image, DEPTH_TOPIC,
            lambda m: self.frames.append(self.b.imgmsg_to_cv2(m,"passthrough")), 10)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC,
            lambda m: setattr(self, "k", m.k), 10)

rclpy.init(); t=T()
end=time.monotonic()+10
while time.monotonic()<end and len(t.frames)<40: rclpy.spin_once(t,timeout_sec=0.1)
t.destroy_node(); rclpy.shutdown()
if not t.frames: print("кадров нет"); sys.exit(1)
FX = float(t.k[0]) if t.k is not None else 584.1   # из camera_info; запасное значение — см. STATUS §2

st = np.stack([f.astype(np.float32) for f in t.frames])
st[st==0]=np.nan
d = np.nanmedian(st, axis=0)
h,w = d.shape
yy,xx = np.mgrid[0:h,0:w]
r = np.hypot(xx-CX, yy-CY)
inside = r < R

say(f"кадров: {len(t.frames)}   рабочий круг r<{R:.0f} px вокруг ({CX:.0f},{CY:.0f})")
v = d[inside & np.isfinite(d)]
say(f"\nвнутри круга: {v.size}/{int(inside.sum())} валидных ({100*v.size/inside.sum():.1f}%)")
say(f"  глубина: min={v.min():.0f}  max={v.max():.0f}  median={np.median(v):.0f} мм")
say(f"  размах {v.max()-v.min():.0f} мм")

def fit_plane(mask, trims=3, keep_mm=6.0):
    """Устойчивая подгонка плоскости: отбрасываем то, что на неё не ложится.

    Обычный МНК тянется за любым предметом в круге. Из-за этого отпечаток
    камеры срабатывал ложно, стоило положить на стол доску: её толщина
    наклоняла подгонку сильнее порога. Здесь плоскость подгоняется несколько
    раз, и точки дальше keep_mm от неё в следующий заход не берутся — остаётся
    доминирующая поверхность, то есть стол.
    """
    use = mask.copy()
    coef = None
    for _ in range(trims + 1):
        A = np.c_[xx[use], yy[use], np.ones(use.sum())]
        coef, *_ = np.linalg.lstsq(A, d[use], rcond=None)
        dev = np.abs(d - (coef[0]*xx + coef[1]*yy + coef[2]))
        nxt = mask & np.isfinite(dev) & (dev < keep_mm)
        if nxt.sum() < 200 or nxt.sum() == use.sum():
            break
        use = nxt
    return coef, use


# подгонка плоскости внутри круга -> что не стол
m = inside & np.isfinite(d)
coef, used = fit_plane(m)
A = np.c_[xx[m], yy[m], np.ones(m.sum())]
res = d[m] - A@coef
dropped = int(m.sum() - used.sum())
if dropped > 0.005 * m.sum():
    say(f"\nустойчивая подгонка: {dropped} точек ({100*dropped/m.sum():.1f}%) "
        f"отброшены как не принадлежащие столу")
say(f"\nплоскость по кругу: остаток median|r|={np.median(np.abs(res)):.2f} мм, "
      f"p95={np.percentile(np.abs(res),95):.2f}, max={np.abs(res).max():.0f}")

# Наклон стола к оптической оси. Коэффициенты идут в мм на пиксель, физический
# уклон получается делением на размер пикселя в плоскости стола: z/fx мм на px.
z0 = coef[0]*CX + coef[1]*CY + coef[2]
mm_per_px = z0 / FX
tilt_x = np.degrees(np.arctan(coef[0] / mm_per_px))
tilt_y = np.degrees(np.arctan(coef[1] / mm_per_px))
say(f"  наклон стола к оптической оси: {tilt_x:+.2f}° по x, {tilt_y:+.2f}° по y "
      f"(итого {np.hypot(tilt_x, tilt_y):.2f}°)")
out = np.abs(res) > 15
say(f"  пикселей дальше 15 мм от плоскости: {out.sum()} ({100*out.mean():.2f}%)")

# где именно выбросы
if out.sum():
    ox, oy = xx[m][out], yy[m][out]
    say(f"  их разброс: x {ox.min():.0f}..{ox.max():.0f}, y {oy.min():.0f}..{oy.max():.0f}")

# насколько близко край стола / база робота
far = np.isfinite(d) & (d > np.median(v)+100)
if far.any():
    rr = r[far]
    say(f"\nпиксели глубже медианы на 100+ мм (пол, край стола): "
          f"ближайший на r={rr.min():.0f} px от главной точки")
    say(f"  запас до границы круга: {rr.min()-R:+.0f} px")
else:
    say("\nдалёких пикселей (пол/край) в кадре нет")

nan_in = inside & ~np.isfinite(d)
if nan_in.sum():
    say(f"\nдыры внутри круга: {nan_in.sum()} px ({100*nan_in.sum()/inside.sum():.1f}%), "
          f"ближайшая на r={r[nan_in].min():.0f}")
    say("карта дыр по блокам 40x40 px ('.' = чисто, цифра = десятки % дыр,")
    say("'#' = блок целиком вне рабочего круга):")
    for by in range(0, h, 40):
        row = []
        for bx in range(0, w, 40):
            ins = inside[by:by+40, bx:bx+40]
            if not ins.any():
                row.append("#"); continue
            frac = (nan_in[by:by+40, bx:bx+40] & ins).sum() / ins.sum()
            row.append("." if frac < 0.05 else str(min(9, int(frac*10))))
        say("   " + " ".join(row))
    say("   чёрные поверхности ИК не отражают — так выглядит ChArUco-доска")
    say("   или тёмный предмет, попавший в зону")
else:
    say("\nдыр внутри круга нет")

# Отпечаток: коэффициенты плоскости стола в кадре. Если камеру заденут после
# закрепления, повторный прогон это покажет — hand-eye после сдвига недействителен.
if "--save" in sys.argv:
    from suction.config import DATA_CALIB
    import json, datetime
    out = DATA_CALIB / "zone_baseline.json"
    ref = {
        "снято": datetime.datetime.now().isoformat(timespec="seconds"),
        "плоскость_a_b_c": [float(x) for x in coef],
        "fx": FX,
        "глубина_в_главной_точке_мм": float(z0),
        "наклон_град_x_y": [float(tilt_x), float(tilt_y)],
        "остаток_median_мм": float(np.median(np.abs(res))),
        "остаток_p95_мм": float(np.percentile(np.abs(res), 95)),
        "дыр_в_круге_px": int(nan_in.sum()),
        "запас_до_далёких_px": float(r[far].min() - R) if far.any() else None,
    }
    out.write_text(json.dumps(ref, ensure_ascii=False, indent=2))
    say(f"\nотпечаток сохранён: {out}")
    say("после закрепления камеры прогони с --save ещё раз — это эталон,")
    say("с которым сверяться, если камеру заденут")
else:
    prev = None
    try:
        from suction.config import DATA_CALIB
        import json
        prev = json.loads((DATA_CALIB / "zone_baseline.json").read_text())
    except Exception:
        pass
    if prev:
        dz = z0 - prev["глубина_в_главной_точке_мм"]
        old = prev.get("наклон_град_x_y")
        if old is None:   # эталон снят до того, как углы стали писаться
            pf = prev.get("fx", FX)
            pz = prev["глубина_в_главной_точке_мм"]
            old = [float(np.degrees(np.arctan(c / (pz / pf))))
                   for c in prev["плоскость_a_b_c"][:2]]
        da, db = tilt_x - old[0], tilt_y - old[1]
        # Пороги по измеренному разбросу повторных прогонов на неподвижной
        # камере: глубина уходила на 0.1-0.3 мм, наклон до 0.29°. Берём с
        # запасом, иначе проверка ловит собственный шум.
        moved = abs(dz) > 3 or abs(da) > 0.6 or abs(db) > 0.6
        say(f"\nсверка с эталоном от {prev['снято']}:")
        say(f"  сдвиг по глубине {dz:+.1f} мм, наклон {da:+.2f}° / {db:+.2f}°")
        if moved:
            print(f"  КАМЕРУ СДВИНУЛИ относительно эталона от {prev['снято']}: "
                  f"глубина {dz:+.1f} мм, наклон {da:+.2f}° / {db:+.2f}°")
            print("  Калибровка после этого недействительна. Порядок пересъёмки —"
                  " docs/calibration.md")
            sys.exit(2)
        say("  в пределах шума, камера на месте")
        if QUIET:
            print(f"  камера на месте (сдвиг {dz:+.1f} мм, "
                  f"наклон {da:+.2f}°/{db:+.2f}°)")
