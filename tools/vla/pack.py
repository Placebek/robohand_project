#!/usr/bin/env python3
"""Сырые эпизоды -> LeRobotDataset. Второй шаг записи демонстраций (Трек А).

Запускается ДРУГИМ интерпретатором, чем рекордер: .venv-lerobot (numpy 2,
opencv-python-headless, torch CPU). В общий .venv стенда lerobot ставить
нельзя — он потребует numpy 2 и headless-сборку cv2, а на них не заводится
cv_bridge и не открывается ни одно окно OpenCV. Разбор — шапка
suction/vla/raw.py, запуск — tools/vla/pack.sh.

Что здесь происходит с данными, помимо перекладывания:
  - BGR -> RGB и приведение к schema.IMAGE_HEIGHT x IMAGE_WIDTH. Сырое
    хранится как отдало железо, приведение к схеме — забота этого шага;
  - action[t] = state[t+1], последний кадр эпизода отбрасывается
    (raw.build_actions, объяснение там же);
  - finalize() в конце. Без него датасет остаётся в состоянии записи и
    не читается вообще: LeRobotDataset.__getitem__ бросает
    "Cannot read from a dataset that is being recorded" — то есть обучение
    на workstation упадёт не при первом батче, а при первом же обращении.

Повторный запуск безопасен: упакованные эпизоды помечены packed.json и
пропускаются. Перепаковать всё с нуля — удалить папку датасета в
data/vla_datasets/ и запустить с --force.

Запуск: tools/vla/pack.sh [--repo-id local/e6-suction-pickplace-v1]
"""
import argparse
import sys
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np   # noqa: E402

from suction.vla import raw, schema           # noqa: E402
from suction.vla.dataset_paths import default_root  # noqa: E402

try:
    import cv2
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError as e:
    print(f"нет lerobot или cv2 в этом интерпретаторе: {e}\n")
    print("Упаковка идёт из .venv-lerobot, а не из .venv стенда.")
    print("Запускать так:  tools/vla/pack.sh")
    print("Если venv ещё нет — docs/setup.md §7.")
    sys.exit(2)


def load_frame(path):
    """JPEG с диска -> RGB нужного размера, как ждёт схема датасета."""
    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f"кадр не читается: {path}")
    if bgr.shape[:2] != (schema.IMAGE_HEIGHT, schema.IMAGE_WIDTH):
        bgr = cv2.resize(bgr, (schema.IMAGE_WIDTH, schema.IMAGE_HEIGHT))
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def open_dataset(repo_id, root):
    """Новый датасет либо дозапись в существующий по тому же (repo_id, root).

    LeRobotDataset.create() создаёт директорию с exist_ok=False и падает
    FileExistsError, если она уже есть — это и есть признак «датасет уже
    существует», отдельно проверять наличие файлов не нужно. resume()
    работает и после finalize(), проверено на 0.6.1.
    """
    try:
        ds = LeRobotDataset.create(
            repo_id, schema.FPS,
            features=schema.build_dataset_features(),
            root=root, robot_type=schema.ROBOT_TYPE, use_videos=True,
        )
        print(f"новый датасет: {repo_id} в {ds.root}")
        return ds
    except FileExistsError:
        ds = LeRobotDataset.resume(repo_id, root=root)
        print(f"датасет существует, дозаписываю: {repo_id} "
              f"({ds.meta.total_episodes} эп., {ds.meta.total_frames} кадров)")
        return ds


def check_episode(meta, ep_dir):
    """Причина не брать эпизод, или None если он годен."""
    if meta["fps"] != schema.FPS:
        return (f"записан на {meta['fps']} Гц, а датасет на {schema.FPS} — "
                f"смешивать частоты нельзя")
    if list(meta["joint_names"]) != list(schema.JOINT_NAMES):
        return f"другие поля состояния: {meta['joint_names']}"
    if len(meta["frames"]) < 2:
        return f"{len(meta['frames'])} кадров, нужно минимум 2"
    for fr in meta["frames"]:
        if len(fr["state"]) != len(schema.JOINT_NAMES):
            return f"кадр {fr['i']}: состояние длины {len(fr['state'])}"
        if not raw.frame_path(ep_dir, fr["i"]).exists():
            return f"нет файла кадра {fr['i']}"
    return None


def pack_episode(ds, ep_dir, meta):
    states = [fr["state"] for fr in meta["frames"]]
    pairs = raw.build_actions(states)
    for i, (state, action) in enumerate(pairs):
        ds.add_frame({
            "observation.state": np.array(state, dtype=np.float32),
            f"observation.images.{schema.CAMERA_KEY}":
                load_frame(raw.frame_path(ep_dir, meta["frames"][i]["i"])),
            "action": np.array(action, dtype=np.float32),
            "task": meta["task"],
        })
    ds.save_episode()
    return len(pairs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-id", default=schema.DEFAULT_REPO_ID,
                    help=f"какой датасет собирать (по умолчанию {schema.DEFAULT_REPO_ID})")
    ap.add_argument("--force", action="store_true",
                    help="брать и уже упакованные эпизоды (после удаления "
                         "папки датасета — пересборка с нуля)")
    ap.add_argument("--dry-run", action="store_true",
                    help="только показать, что было бы упаковано")
    args = ap.parse_args()

    root = default_root(args.repo_id)
    episodes = raw.list_episodes(args.repo_id, include_packed=args.force)

    print(f"датасет: {args.repo_id}")
    print(f"сырьё:   {raw.raw_root(args.repo_id)}")
    print(f"путь:    {root}")
    print(f"к упаковке: {len(episodes)} эпизодов\n")

    if not episodes:
        print("нечего упаковывать. Записать демонстрации: tools/vla/record.sh")
        return 0

    if args.dry_run:
        for d in episodes:
            meta = raw.read_meta(d)
            bad = check_episode(meta, d)
            mark = f"ОТКАЗ ({bad})" if bad else f"{len(meta['frames']) - 1} шагов"
            print(f"  {d.name}  {meta['task']!r}  {mark}")
        return 0

    ds = open_dataset(args.repo_id, root)
    packed = skipped = frames = 0

    for d in episodes:
        meta = raw.read_meta(d)
        bad = check_episode(meta, d)
        if bad:
            print(f"  {d.name}: ПРОПУСК — {bad}")
            skipped += 1
            continue
        n = pack_episode(ds, d, meta)
        raw.mark_packed(d, ds.root, ds.meta.total_episodes - 1)
        packed += 1
        frames += n
        print(f"  {d.name}: {n} шагов, задача {meta['task']!r}")

    print("\nfinalize()...")
    ds.finalize()

    print(f"\nупаковано эпизодов: {packed}, пропущено: {skipped}, "
          f"кадров добавлено: {frames}")
    print(f"итого в датасете: {ds.meta.total_episodes} эп., "
          f"{ds.meta.total_frames} кадров, задач: {len(ds.meta.tasks)}")
    print(f"датасет: {ds.root}")
    print("\nна workstation:  rsync -av data/vla_datasets/ "
          "workstation:~/robohand/data/vla_datasets/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
