#!/usr/bin/env python3
"""Запись демонстраций для VLA (Трек А): кинестетическое ведение + камера.

Человек держит руку за корпус, drag-teach делает её податливой, ведёт через
подбор -> перенос -> укладку, вручную дёргает вакуум клавишей. Скрипт пишет
углы суставов, состояние присоски и кадр RGB на suction.vla.schema.FPS в
СЫРОЙ формат эпизода (suction/vla/raw.py): JPEG-кадры плюс meta.json.

В LeRobotDataset это превращает отдельный шаг — `tools/vla/pack.sh`. Так
сделано не из любви к промежуточным форматам: lerobot требует numpy 2 и
headless-сборку OpenCV, а тут нужны numpy 1 (под него собран cv_bridge) и
системный cv2 с окном. Подробный разбор — в шапке suction/vla/raw.py.

Управление в окне камеры:
  R или ПРОБЕЛ   начать / закончить эпизод
  G              вкл/выкл вакуум (действует только во время эпизода)
  BACKSPACE      отменить текущий эпизод (испортили демонстрацию)
  Q              выход, текущий незаконченный эпизод отменяется

Кадры уходят на диск сразу, а не копятся в памяти: полминуты записи при
10 Гц — это 276 МБ несжатых кадров, и на длинной демонстрации ноутбук
кончится раньше, чем эпизод.

Запуск: tools/vla/record.sh --task "подними жёлтый кубик и положи в коробку"
Дальше: tools/vla/pack.sh   (собрать датасет из накопленного сырья)
"""
import argparse
import shutil
import sys
import time
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "suction").is_dir())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2                      # noqa: E402
import rclpy                    # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node     # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from suction.arm.client import ArmClient, ArmError, run_tool  # noqa: E402
from suction.config import RGB_TOPIC  # noqa: E402
from suction.vla import raw, schema  # noqa: E402

WINDOW = "vla record - R start/stop, G vacuum, BACKSPACE cancel, Q vyhod"


class Camera(Node):
    def __init__(self):
        super().__init__("vla_record_camera")
        self.bridge = CvBridge()
        self.rgb = None
        self.create_subscription(Image, RGB_TOPIC,
                                 lambda m: setattr(self, "rgb",
                                                   self.bridge.imgmsg_to_cv2(m, "bgr8")),
                                 10)

    def wait_ready(self, timeout=10.0):
        end = time.monotonic() + timeout
        while self.rgb is None:
            if time.monotonic() > end:
                raise RuntimeError("нет кадров RGB — драйвер камеры запущен?")
            rclpy.spin_once(self, timeout_sec=0.1)

    def pump(self):
        rclpy.spin_once(self, timeout_sec=0.01)


class Episode:
    """Один сырой эпизод: папка на диске, куда кадры ложатся по мере съёмки."""

    def __init__(self, repo_id, task):
        self.repo_id = repo_id
        self.task = task
        self.episode_id = raw.new_episode_id()
        self.dir = raw.episode_dir(repo_id, self.episode_id)
        (self.dir / raw.FRAMES_SUBDIR).mkdir(parents=True, exist_ok=True)
        self.frames = []
        self.t0 = time.monotonic()

    def add(self, bgr, state):
        i = len(self.frames)
        cv2.imwrite(str(raw.frame_path(self.dir, i)), bgr,
                    [cv2.IMWRITE_JPEG_QUALITY, raw.JPEG_QUALITY])
        self.frames.append({"i": i,
                            "t": round(time.monotonic() - self.t0, 4),
                            "state": [round(float(v), 4) for v in state]})

    def __len__(self):
        return len(self.frames)

    def close(self):
        """Дописать meta.json. False, если кадров слишком мало — папка удалена."""
        if len(self.frames) < 2:
            print(f"  эпизод отброшен: {len(self.frames)} кадров, нужно минимум 2")
            self.discard()
            return False
        raw.write_meta(self.dir, self.repo_id, self.episode_id, self.task,
                       schema.FPS, schema.ROBOT_TYPE, schema.JOINT_NAMES,
                       self.frames)
        dur = len(self.frames) / schema.FPS
        print(f"  эпизод записан: {len(self.frames)} кадров ({dur:.1f} с) -> "
              f"{self.dir.name}")
        return True

    def discard(self):
        shutil.rmtree(self.dir, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True,
                    help="инструкция задачи, пишется в каждый кадр эпизода "
                         "(строка 'task' формата LeRobot)")
    ap.add_argument("--repo-id", default=schema.DEFAULT_REPO_ID,
                    help=f"идентификатор датасета (по умолчанию {schema.DEFAULT_REPO_ID})")
    ap.add_argument("--sensitivity", type=int, default=None,
                    help="податливость drag-teach, 1-90. Меньше — больше "
                         "сопротивление. Без флага — заводская")
    args = ap.parse_args()

    raw_dir = raw.raw_root(args.repo_id)
    raw_dir.mkdir(parents=True, exist_ok=True)
    already = len(raw.list_episodes(args.repo_id, include_packed=True))
    print(f"датасет: {args.repo_id}")
    print(f"сырьё:   {raw_dir}  (уже записано эпизодов: {already})")
    print(f"задача:  {args.task!r}\n")

    print("=" * 66)
    print("РУКА СТАНЕТ ПОДАТЛИВОЙ (drag-teach). ДЕРЖИ ЕЁ, прежде чем")
    print("соглашаться — она обмякнет и просядет под своим весом.")
    print("=" * 66)
    if input("\nПоехали? y/n: ").strip().lower() not in ("y", "д", "да"):
        return 0

    rclpy.init()
    cam = Camera()
    episodes_saved = 0

    try:
        cam.wait_ready()

        with ArmClient() as arm:
            print(f"рука: режим {arm.mode()} — {arm.mode_name()}")
            arm.enable()

            dragging = False
            try:
                arm.drag(True, sensitivity=args.sensitivity)
                dragging = True
                print("рука податлива — води её руками\n")
            except ArmError as e:
                print(f"  StartDrag не прошёл: {e}")
                print("  Веди рукой через кнопку ручного ведения на фланце —")
                print("  скрипт просто читает, где рука оказалась.\n")

            episode = None
            gripper_on = arm.gripper_state()
            next_sample = 0.0
            period = 1.0 / schema.FPS

            try:
                cv2.namedWindow(WINDOW)
                print("R/ПРОБЕЛ — начать эпизод. Веди руку через задачу.\n")

                while True:
                    cam.pump()
                    if cam.rgb is None:
                        continue
                    now = time.monotonic()

                    if episode is not None and now >= next_sample:
                        state = schema.state_vector(arm.joints(), gripper_on)
                        episode.add(cam.rgb, state)
                        next_sample += period
                        if next_sample < now:   # пропустили такт — не копим долг
                            next_sample = now + period

                    view = cam.rgb.copy()
                    # Подписи ТОЛЬКО латиницей: putText рисует шрифтом Hershey,
                    # кириллицы в нём нет — вместо букв идут "?".
                    status = (f"REC {len(episode)} frames" if episode is not None
                              else "ready (R = start)")
                    color = (0, 0, 220) if episode is not None else (200, 200, 200)
                    cv2.putText(view, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                               0.6, color, 2, cv2.LINE_AA)
                    cv2.putText(view, f"vacuum: {'ON' if gripper_on else 'off'}   "
                                      f"saved: {episodes_saved}",
                               (8, view.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                               0.5, (0, 220, 0), 1, cv2.LINE_AA)
                    cv2.imshow(WINDOW, view)
                    key = cv2.waitKey(1) & 0xFF

                    if key in (ord("r"), ord(" ")):
                        if episode is None:
                            episode = Episode(args.repo_id, args.task)
                            next_sample = time.monotonic()
                            print(f"запись начата: {episode.episode_id}")
                        else:
                            if episode.close():
                                episodes_saved += 1
                            episode = None
                    elif key == 8:           # BACKSPACE
                        if episode is not None:
                            episode.discard()
                            episode = None
                            print("эпизод отменён")
                    elif key == ord("g"):
                        gripper_on = not gripper_on
                        arm.gripper(gripper_on)
                    elif key == ord("q"):
                        if episode is not None:
                            print("выход посреди записи — эпизод отменён")
                            episode.discard()
                        break
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
        # выход из "with ArmClient()" сам вызывает disable() и закрывает сокет
    finally:
        cam.destroy_node()
        rclpy.shutdown()
        print(f"\nзаписано эпизодов за сессию: +{episodes_saved}")
        print(f"сырьё на диске: {raw_dir}")
        if episodes_saved:
            print("собрать датасет: tools/vla/pack.sh "
                  f"--repo-id {args.repo_id}")
    return 0


if __name__ == "__main__":
    sys.exit(run_tool(main))
