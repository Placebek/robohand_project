#!/bin/bash
# Первое движение E6. РУКА ПОЕДЕТ. Человек у аварийного стопа обязателен.
# Обёртка чистит snap-переменные VS Code, как и остальные run_*.sh.
set -e
cd "$(dirname "$(readlink -f "$0")")/.."

exec env -u SNAP -u SNAP_LIBRARY_PATH -u SNAP_NAME -u SNAP_REVISION -u SNAP_DATA \
    -u SNAP_USER_DATA -u SNAP_USER_COMMON -u SNAP_REAL_HOME -u LD_LIBRARY_PATH \
    -u PYTHONPATH \
    python3 tools/first_motion_check.py "$@"
