"""Сырой эпизод демонстрации: что пишет рекордер и что читает упаковщик.

Почему между записью и LeRobotDataset появился промежуточный формат, а не
add_frame прямо в цикле записи, как обещал первый вариант плана: lerobot 0.6.1
требует numpy>=2.0 и opencv-python-headless, а cv_bridge из ROS Jazzy собран
под numpy 1.x — в его cv_bridge_boost.so зашито numpy.core._multiarray_umath
(в numpy 2 это numpy._core), и под numpy 2 он не импортируется вовсе. Плюс
headless-сборка cv2 не умеет imshow, а окно с клавишами R/G/BACKSPACE — весь
интерфейс записи. Один интерпретатор обе стороны не вмещает, поэтому:

    tools/vla/record.sh  (.venv, ROS, numpy 1, системный cv2 с GUI) -> сырое
    tools/vla/pack.sh    (.venv-lerobot, numpy 2, torch CPU)  сырое -> датасет

Побочная выгода важнее самой развязки: демонстрация — это физический труд
человека у стенда, и терять серию из-за исключения в чужой библиотеке посреди
записи нельзя. Сырое лежит на диске и перепаковывается сколько угодно раз —
после обновления lerobot, смены FPS-политики или правки схемы полей.

Раскладка на диске:

    data/vla_raw/<repo_id, / -> __>/<episode_id>/
        frames/000000.jpg ...   кадры как их отдала камера, BGR (cv2-родной)
        meta.json               состояния, задача, тайминги
        packed.json             отметка упаковщика: куда и когда ушёл эпизод

meta.json пишется ПОСЛЕДНИМ и атомарно (запись во временный файл + rename).
Его наличие и есть признак целого эпизода: если рекордер убили посреди записи,
папка с кадрами останется, но упаковщик её не возьмёт и не испортит датасет.
"""
import json
import os
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
VLA_RAW_DIR = ROOT / "data" / "vla_raw"

FORMAT = "robohand-vla-raw/1"
FRAMES_SUBDIR = "frames"
META_NAME = "meta.json"
PACKED_NAME = "packed.json"

# Кадры хранятся JPEG, не PNG: LeRobot всё равно перекодирует их в MP4 с
# потерями, так что лишний бит точности умрёт на следующем шаге, а разница в
# объёме — порядок (50 КБ против 400 КБ на кадр, то есть 15 МБ против 120 МБ
# на 30-секундный эпизод при 10 Гц).
JPEG_QUALITY = 95


def raw_root(repo_id):
    """Папка со всеми сырыми эпизодами одного датасета."""
    return VLA_RAW_DIR / repo_id.replace("/", "__")


def new_episode_id():
    """Сортируемый и уникальный в пределах секунды идентификатор эпизода."""
    return datetime.now().strftime("%Y%m%d-%H%M%S")


def episode_dir(repo_id, episode_id):
    return raw_root(repo_id) / episode_id


def frame_path(ep_dir, index):
    return Path(ep_dir) / FRAMES_SUBDIR / f"{index:06d}.jpg"


def write_meta(ep_dir, repo_id, episode_id, task, fps, robot_type,
               joint_names, frames):
    """Закрыть эпизод: записать meta.json атомарно, последним действием.

    frames — список словарей {"i", "t", "state"}. Возвращает путь к meta.json.
    """
    ep_dir = Path(ep_dir)
    meta = {
        "format": FORMAT,
        "repo_id": repo_id,
        "episode_id": episode_id,
        "task": task,
        "fps": fps,
        "robot_type": robot_type,
        "joint_names": list(joint_names),
        "recorded_at": datetime.now().isoformat(timespec="seconds"),
        "n_frames": len(frames),
        "frames": frames,
    }
    final = ep_dir / META_NAME
    tmp = ep_dir / (META_NAME + ".part")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, final)
    return final


def read_meta(ep_dir):
    with open(Path(ep_dir) / META_NAME, encoding="utf-8") as f:
        meta = json.load(f)
    if meta.get("format") != FORMAT:
        raise ValueError(f"{ep_dir}: чужой формат {meta.get('format')!r}, "
                         f"ожидался {FORMAT!r}")
    return meta


def is_complete(ep_dir):
    return (Path(ep_dir) / META_NAME).exists()


def is_packed(ep_dir):
    return (Path(ep_dir) / PACKED_NAME).exists()


def mark_packed(ep_dir, dataset_root, episode_index):
    """Отметить, что эпизод уже ушёл в датасет — чтобы не задвоить при повторе."""
    with open(Path(ep_dir) / PACKED_NAME, "w", encoding="utf-8") as f:
        json.dump({"dataset_root": str(dataset_root),
                   "episode_index": episode_index,
                   "packed_at": datetime.now().isoformat(timespec="seconds")},
                  f, ensure_ascii=False, indent=1)


def list_episodes(repo_id, include_packed=False):
    """Целые эпизоды по возрастанию времени записи; недописанные пропускаются."""
    root = raw_root(repo_id)
    if not root.is_dir():
        return []
    out = []
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        if not is_complete(d):
            continue
        if is_packed(d) and not include_packed:
            continue
        out.append(d)
    return out


def build_actions(states):
    """action[t] = state[t+1]. Последний кадр действия не имеет — отбрасывается.

    При кинестетическом ведении нет команды отдельно от траектории: человек
    ведёт руку рукой, есть только то, куда она в итоге пришла. Стандартная
    практика для таких демонстраций — брать следующим состоянием.

    Возвращает список пар (state, action), на один короче входа.
    """
    return [(states[i], states[i + 1]) for i in range(len(states) - 1)]
