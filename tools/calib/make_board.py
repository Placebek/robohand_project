#!/usr/bin/env python3
"""Печатная ChArUco-доска для hand-eye калибровки (план §5.3).

Почему ChArUco, а не шахматка и не одиночный ArUco: шахматка требует, чтобы
доска целиком попала в кадр и не была перекрыта, а одиночный маркер даёт
плохую оценку поворота вокруг своей нормали. ChArUco сочетает субпиксельную
точность углов шахматки с устойчивостью к частичному перекрытию — на настольной
сцене, где доска ездит по краю кадра, это решающе.

Печать — единственный шаг калибровки, который можно сделать до подключения
робота. Масштаб критичен: ошибка печати в 2% даёт систематическую ошибку
позиционирования в те же 2%, и никакой алгоритм её не обнаружит — она уедет
прямо в результаты.

Рендер идёт через PIL попиксельно на холст A4 нужного DPI: matplotlib при
вставке крупного битмапа в PDF ресемплит его и разрушает мелкие биты маркеров.

Запуск: python3 tools/calib/make_board.py
"""
import sys
from pathlib import Path

import cv2
import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
sys.path.insert(0, str(ROOT))
from suction.config import DATA_CALIB  # noqa: E402

# --- Параметры доски ------------------------------------------------------
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_MM = 30.0          # 5x7x30мм = 150x210 мм: влезает на A4 с полями
MARKER_MM = 22.0          # ~0.73 от клетки — рекомендованное соотношение
DICT_NAME = "DICT_5X5_250"
DPI = 300                 # 300 dpi: бит маркера ~37 px, лазерник отрисует чисто

A4_W_MM, A4_H_MM = 210.0, 297.0
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def mm2px(mm):
    return int(round(mm / 25.4 * DPI))


def main():
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, DICT_NAME))
    board = cv2.aruco.CharucoBoard_create(
        SQUARES_X, SQUARES_Y, SQUARE_MM / 1000.0, MARKER_MM / 1000.0, dictionary
    )

    board_w_mm = SQUARES_X * SQUARE_MM
    board_h_mm = SQUARES_Y * SQUARE_MM
    board_px = (mm2px(board_w_mm), mm2px(board_h_mm))
    board_img = Image.fromarray(board.draw(board_px)).convert("L")

    DATA_CALIB.mkdir(parents=True, exist_ok=True)
    png_path = DATA_CALIB / "charuco_board.png"
    board_img.save(png_path, dpi=(DPI, DPI))

    # --- Холст A4 в натуральную величину ----------------------------------
    page = Image.new("L", (mm2px(A4_W_MM), mm2px(A4_H_MM)), 255)
    x0_mm = (A4_W_MM - board_w_mm) / 2
    y0_mm = 20.0                                   # снизу остаётся место под линейку
    page.paste(board_img, (mm2px(x0_mm), mm2px(y0_mm)))

    draw = ImageDraw.Draw(page)
    font = ImageFont.truetype(FONT_PATH, mm2px(3.0))
    small = ImageFont.truetype(FONT_PATH, mm2px(2.4))

    # Контрольная линейка 100 мм: после печати померь штангенциркулем.
    # Не 100.0 -> в диалоге печати стоит "вписать в страницу"; переставь на
    # "100% / фактический размер" и печатай заново.
    ry_mm = A4_H_MM - 30.0
    rx0_mm = (A4_W_MM - 100.0) / 2
    draw.line([(mm2px(rx0_mm), mm2px(ry_mm)), (mm2px(rx0_mm + 100.0), mm2px(ry_mm))],
              fill=0, width=max(1, mm2px(0.4)))
    for x_mm in (rx0_mm, rx0_mm + 50.0, rx0_mm + 100.0):
        draw.line([(mm2px(x_mm), mm2px(ry_mm - 3.0)), (mm2px(x_mm), mm2px(ry_mm + 3.0))],
                  fill=0, width=max(1, mm2px(0.4)))
    draw.text((mm2px(A4_W_MM / 2), mm2px(ry_mm - 6.0)),
              "100.0 мм — проверь штангенциркулем", fill=0, font=font, anchor="ms")
    draw.text((mm2px(A4_W_MM / 2), mm2px(ry_mm + 9.0)),
              f"ChArUco {SQUARES_X}x{SQUARES_Y}   клетка {SQUARE_MM:g} мм   "
              f"маркер {MARKER_MM:g} мм   {DICT_NAME}",
              fill=90, font=small, anchor="ms")

    pdf_path = DATA_CALIB / "charuco_board_A4.pdf"
    page.save(pdf_path, "PDF", resolution=DPI)

    # --- Спека: калибровочный скрипт читает её, а не хардкодит размеры -----
    spec = {
        "squares_x": SQUARES_X,
        "squares_y": SQUARES_Y,
        "square_length_m": SQUARE_MM / 1000.0,
        "marker_length_m": MARKER_MM / 1000.0,
        "dictionary": DICT_NAME,
        "board_size_mm": [board_w_mm, board_h_mm],
        "measured_square_mm": None,   # ВПИШИ реальный замер после печати
    }
    (DATA_CALIB / "charuco_board.yaml").write_text(
        yaml.safe_dump(spec, allow_unicode=True, sort_keys=False))

    print(f"PDF (печатать 100%, БЕЗ 'вписать в страницу'): {pdf_path}")
    print(f"PNG:                                           {png_path}")
    print(f"Спека:                                         {DATA_CALIB/'charuco_board.yaml'}")
    print(f"\nДоска {board_w_mm:g}x{board_h_mm:g} мм, клетка {SQUARE_MM:g} мм.")
    print("После печати померь клетку штангенциркулем и впиши в measured_square_mm — "
          "калибровка пойдёт по замеру, а не по номиналу.")


if __name__ == "__main__":
    main()
