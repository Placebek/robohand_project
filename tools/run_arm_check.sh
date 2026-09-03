#!/bin/bash
# Проверка связи с рукой. ROS и Qt здесь не нужны, но snap-переменные VS Code
# всё равно убираются — ради единообразия с остальными обёртками.
set -e
cd "$(dirname "$(readlink -f "$0")")/.."

exec env -u SNAP -u SNAP_LIBRARY_PATH -u SNAP_NAME -u SNAP_REVISION -u SNAP_DATA \
    -u SNAP_USER_DATA -u SNAP_USER_COMMON -u SNAP_REAL_HOME -u LD_LIBRARY_PATH \
    -u PYTHONPATH \
    python3 tools/check_arm_connection.py "$@"
