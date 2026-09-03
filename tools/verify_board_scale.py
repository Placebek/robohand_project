#!/usr/bin/env python3
"""Проверка масштаба напечатанной ChArUco-доски.

Домашние принтеры почти никогда не держат масштаб точно. Это НЕ повод
перепечатывать: калибровке безразличен номинал клетки, ей важно знать
фактический размер. Ошибка масштаба фатальна, только если она
неодинакова по осям — тогда сетка перестаёт быть квадратной, а
cv2.aruco.CharucoBoard_create умеет задать только одну длину клетки.

Отсюда процедура: померить пролёт по обеим осям, сравнить масштабы.
  * расходятся меньше чем на 0.2% -> печать годная, пишем замер в yaml;
  * больше -> печать анизотропная, нужен другой принтер.

Меряй пролёт через ВСЕ клетки сразу, а не одну: погрешность штангенциркуля
±0.05 мм на 150 мм даёт 0.03%, а на одной клетке 30 мм — уже 0.17%,
то есть на уровне самого искомого эффекта.

Запуск:
  python3 tools/verify_board_scale.py --x 149.4 --y 209.1
где --x пролёт по 5 клеткам горизонтали, --y по 7 клеткам вертикали
(измеряй от внешнего края первой клетки до внешнего края последней).
"""
import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from suction.config import DATA_CALIB  # noqa: E402

ANISOTROPY_LIMIT = 0.002      # 0.2%


def main():
    spec_path = DATA_CALIB / "charuco_board.yaml"
    spec = yaml.safe_load(spec_path.read_text())
    nominal = spec["square_length_m"] * 1000.0
    nx, ny = spec["squares_x"], spec["squares_y"]

    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, required=True,
                    help=f"замеренный пролёт по {nx} клеткам горизонтали, мм "
                         f"(номинал {nx * nominal:g})")
    ap.add_argument("--y", type=float, required=True,
                    help=f"замеренный пролёт по {ny} клеткам вертикали, мм "
                         f"(номинал {ny * nominal:g})")
    ap.add_argument("--square", type=float, default=None,
                    help="контрольный замер ОДНОЙ клетки, мм. Третье независимое "
                         "измерение: если один из пролётов промахнулся, оно покажет, какой")
    args = ap.parse_args()

    sq_x = args.x / nx
    sq_y = args.y / ny
    scale_x = sq_x / nominal
    scale_y = sq_y / nominal
    anisotropy = abs(scale_x - scale_y)

    print(f"номинал клетки:     {nominal:.3f} мм")
    print(f"по горизонтали:     {sq_x:.3f} мм  (масштаб {scale_x*100:.2f}%)")
    print(f"по вертикали:       {sq_y:.3f} мм  (масштаб {scale_y*100:.2f}%)")
    print(f"анизотропия:        {anisotropy*100:.3f}%  (предел {ANISOTROPY_LIMIT*100:.1f}%)")

    # Перекрёстная проверка. Пролёт через много клеток точнее одиночного замера,
    # но промахнуться можно и в нём: у ChArUco крайние клетки в чётных рядах
    # белые, и внешний край такого ряда на белой бумаге просто не виден.
    # Замер одной клетки — третий голос, который показывает, какой из двух
    # пролётов ошибочный.
    if args.square is not None:
        d_x = abs(sq_x - args.square)
        d_y = abs(sq_y - args.square)
        print(f"контроль по клетке: {args.square:.3f} мм  "
              f"(расхождение: с горизонталью {d_x:.3f}, с вертикалью {d_y:.3f})")
        if anisotropy > ANISOTROPY_LIMIT and abs(d_x - d_y) > 0.2:
            odd = "горизонтали" if d_x > d_y else "вертикали"
            good = "вертикали" if d_x > d_y else "горизонтали"
            print()
            print(f"ПОХОЖЕ НА ОШИБКУ ЗАМЕРА, А НЕ НА АНИЗОТРОПИЮ.")
            print(f"Замер по {good} согласуется с контрольной клеткой, по {odd} — нет.")
            print(f"Перемерь пролёт по {odd} и запусти снова. Убедись, что меряешь")
            print(f"ряд, который НАЧИНАЕТСЯ И КОНЧАЕТСЯ ЧЁРНОЙ клеткой, — у белой")
            print(f"крайней клетки внешнего края на бумаге не видно.")
            return 2
    print()

    if anisotropy > ANISOTROPY_LIMIT:
        print("ПЕЧАТЬ НЕ ГОДИТСЯ: масштаб по осям разный.")
        print("Сетка перестала быть квадратной, одной длиной клетки её не описать.")
        print("Перепечатай на лазерном принтере — струйники тянут бумагу")
        print("по направлению протяжки и дают именно такую картину.")
        return 1

    measured = (sq_x + sq_y) / 2
    spec["measured_square_mm"] = round(measured, 4)
    spec["measured_span_x_mm"] = args.x
    spec["measured_span_y_mm"] = args.y
    spec_path.write_text(yaml.safe_dump(spec, allow_unicode=True, sort_keys=False))

    print(f"ПЕЧАТЬ ГОДИТСЯ. Масштаб равномерный, отклонение от номинала "
          f"{(measured/nominal - 1)*100:+.2f}% роли не играет.")
    print(f"Записано в {spec_path}: measured_square_mm = {measured:.4f}")
    print("Калибровка будет считаться по этому числу.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
