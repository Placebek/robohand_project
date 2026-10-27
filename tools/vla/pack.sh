#!/bin/bash
# Сырые эпизоды -> LeRobotDataset. Железо не трогает, руку не двигает.
#
# Идёт в ОТДЕЛЬНОМ окружении .venv-lerobot: lerobot требует numpy 2 и
# opencv-python-headless, которые несовместимы с cv_bridge и с окнами OpenCV
# в основном .venv. ROS здесь не нужен вовсе, поэтому профиль plain.
set -e
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/../_lib/env.sh"

VENV="$ROBOHAND_ROOT/.venv-lerobot"
if [ ! -x "$VENV/bin/python3" ]; then
    echo "нет окружения для упаковки: $VENV" >&2
    echo "Как его собрать — docs/setup.md §7." >&2
    exit 2
fi

robohand_env plain
cd "$ROBOHAND_ROOT"
exec env "${ROBOHAND_ENV[@]}" "$VENV/bin/python3" tools/vla/pack.py "$@"
