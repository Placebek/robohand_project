#!/bin/bash
# Управление присоской ES01. Рука не запитывается и не движется.
set -e
cd "$(dirname "$(readlink -f "$0")")/.."
exec env -u SNAP -u SNAP_LIBRARY_PATH -u PYTHONPATH python3 tools/gripper.py "$@"
