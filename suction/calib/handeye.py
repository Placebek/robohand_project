"""Загрузка решения hand-eye с проверкой, что оно не устарело.

Матрица привязана к тому положению камеры, в котором снимался набор поз.
Сдвинули камеру — матрица остаётся синтаксически исправной и молча неверной:
она переводит пиксели в базу, числа выглядят правдоподобно, рука едет мимо.

Так и вышло 07.09.2026: в `handeye.json` лежало решение по набору 16:10, а
камеру после этого переставили. Ошибка в 113 мм по высоте не проявилась ничем
и вскрылась только через доску на столе, которая «взлетела» на 126 мм.

Признак протухания есть, и он дешёвый. Эталон зоны (`zone_baseline.json`)
переснимают ПОСЛЕ каждого сдвига камеры — иначе сверка положения начнёт
ругаться. Значит эталон новее набора поз == камеру двигали после калибровки.

Это предупреждение, а не запрет: эталон могли пересохранить и не двигая
камеру. Решает проверка на реальной сцене — `tools/calib/check.sh --all`.
"""
import json
from pathlib import Path

import numpy as np

from suction.config import DATA_CALIB


def staleness_warning(cal):
    """Строки предупреждения, если камеру двигали после калибровки. Или []."""
    baseline = DATA_CALIB / "zone_baseline.json"
    folder = DATA_CALIB / str(cal.get("набор", ""))
    if not baseline.exists() or not folder.exists():
        return []
    t_base = baseline.stat().st_mtime
    t_set = folder.stat().st_mtime
    if t_base <= t_set:
        return []
    hours = (t_base - t_set) / 3600.0
    return [
        f"ВНИМАНИЕ: эталон зоны на {hours:.1f} ч новее набора "
        f"{cal.get('набор', '?')}.",
        "  Эталон пересохраняют после сдвига камеры, значит камеру, скорее",
        "  всего, двигали ПОСЛЕ этой калибровки. Тогда матрица неверна, и",
        "  ошибка не проявится ничем — сверка положения сравнивает с новым",
        "  эталоном и молчит.",
        "  Проверить: tools/calib/check.sh --all (доска на столе даёт ровный z",
        "  только у верного решения). Пересчитать: tools/calib/solve.sh --data",
        "  <самый свежий набор>.",
    ]


def load_handeye(path=None, quiet=False):
    """Решение hand-eye: (словарь, матрица 4x4 камера->база).

    Печатает предупреждение, если камеру, судя по датам, двигали после
    калибровки. Отключается quiet=True.
    """
    path = Path(path) if path else DATA_CALIB / "handeye.json"
    if not path.exists():
        raise FileNotFoundError(f"нет {path} — сначала tools/calib/solve.sh")
    cal = json.loads(path.read_text(encoding="utf-8"))
    T = np.array(cal["T_камера_в_базу"])
    if not quiet:
        for line in staleness_warning(cal):
            print(line)
    return cal, T
