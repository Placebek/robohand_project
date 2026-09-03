#!/bin/bash
# Решение AX=XB по снятому набору. Робот и камера не нужны.
set -e
cd "$(dirname "$(readlink -f "$0")")/.."
exec env -u SNAP -u SNAP_LIBRARY_PATH -u PYTHONPATH python3 tools/solve_handeye.py "$@"
