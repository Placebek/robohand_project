"""Куда на диске ложатся VLA-датасеты. Отдельно от suction/config.py: тот
модуль не тянет lerobot и остаётся общим для обоих треков, а датасеты — вещь
Трека А."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
VLA_DATASETS_DIR = ROOT / "data" / "vla_datasets"


def default_root(repo_id):
    """Локальный путь для repo_id вида 'namespace/name'.

    LeRobotDataset.create(root=...) кладёт датасет РОВНО в root, ничего не
    дописывая — в отличие от поведения без явного root (тогда подставляется
    $HF_LEROBOT_HOME/{repo_id}). Поэтому namespace добавляем сюда сами, иначе
    два repo_id с одинаковым именем без namespace схлопнутся в одну папку.
    """
    return VLA_DATASETS_DIR / repo_id.replace("/", "__")
