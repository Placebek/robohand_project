"""Наблюдения/действия для VLA-политики (Трек А). Единая точка правды.

Формат — LeRobotDataset (пакет `lerobot`, HuggingFace), совместим с
`lerobot-train` на SmolVLA/ACT без конвертации. Полный план — docs/vla_plan.md.

Модуль читается ОБОИМИ окружениями: и .venv стенда (запись, numpy 1, ROS), и
.venv-lerobot (упаковка, numpy 2). Поэтому lerobot импортируется только внутри
build_dataset_features(), а не на уровне модуля — иначе рекордер не запустится.

Почему именно этот состав полей:
  - state = углы 6 суставов (q_actual, реальные показания энкодеров — см.
    ловушку GetAngle в suction/arm/client.py, НЕ уставка) + состояние
    присоски (0.0/1.0). Поза TCP отдельно не хранится: она однозначно
    выводится из углов прямой кинематикой, дублировать — плодить рассинхрон.
  - action = то же самое, но на следующем шаге. При кинестетическом ведении
    (drag-teach) нет отдельной "команды" — человек ведёт руку рукой, есть
    только пройденная траектория. Стандартная практика для таких
    демонстраций: action[t] = state[t+1]. Пары собираются не при записи, а
    при упаковке — raw.build_actions, вызывается из tools/vla/pack.py.
  - камера одна, RGB. Depth политике НЕ подаётся — смысл VLA-трека в том,
    чтобы действовать по пикселям и языку напрямую, без явной геометрии.
    Depth, нормали и подгонка плоскости остаются в Треке Б.

Резерва под давление здесь сознательно НЕТ. Нулевой столбец в state не несёт
сигнала: политика либо научится его игнорировать (тогда он просто лишний),
либо найдёт ложную корреляцию с шумом инициализации — ни то ни другое не
помогает. Когда датчик появится, версия датасета меняется (repo_id получает
суффикс, например "-pressure"), и state получает СЕДЬМОЕ поле с настоящим,
а не нулевым числом. Старый датасет никуда не девается — это чистая
RGB+proprioception база для сравнения vision-only против vision+pressure,
то есть готовая контрольная группа для RQ3 (см. docs/vla_plan.md).
"""

FPS = 10                   # ограничено камерой (реально ~20-25 Гц, берём
                            # с запасом на джиттер spin_once) и на порядок
                            # выше типовых частот записи VLA-демонстраций
                            # (обычно 5-15 Гц) — гнаться выше некуда и незачем

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6", "gripper"]

CAMERA_KEY = "top"         # HP60C, статичная камера, единственный вид
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640

ROBOT_TYPE = "dobot_e6_es01"
DEFAULT_REPO_ID = "local/e6-suction-pickplace-v1"


def hw_features():
    """Фичи в исходном виде — до перевода в формат LeRobot.

    Возвращает (joints, camera): joints — имя -> float (для observation.state
    и action), camera — имя -> (H, W, C) (только для observation).
    """
    joints = {name: float for name in JOINT_NAMES}
    camera = {CAMERA_KEY: (IMAGE_HEIGHT, IMAGE_WIDTH, 3)}
    return joints, camera


def build_dataset_features():
    """Полный features-словарь для LeRobotDataset.create().

    Камера кладётся ТОЛЬКО в observation: hw_to_dataset_features добавляет
    все элементы с shape-значением как image/video вне зависимости от
    prefix, поэтому дёргаем её отдельным вызовом с одним лишь словарём
    суставов для action — иначе получим паразитный action.images.top.
    """
    from lerobot.utils.constants import ACTION, OBS_STR
    from lerobot.utils.feature_utils import combine_feature_dicts, hw_to_dataset_features

    joints, camera = hw_features()
    obs = hw_to_dataset_features({**joints, **camera}, prefix=OBS_STR, use_video=True)
    act = hw_to_dataset_features(joints, prefix=ACTION, use_video=True)
    return combine_feature_dicts(obs, act)


def state_vector(joint_angles_deg, gripper_on):
    """[j1..j6 (град.), gripper (0/1)] в порядке JOINT_NAMES."""
    if len(joint_angles_deg) != 6:
        raise ValueError(f"нужно 6 углов, получено {len(joint_angles_deg)}")
    return list(joint_angles_deg) + [1.0 if gripper_on else 0.0]
