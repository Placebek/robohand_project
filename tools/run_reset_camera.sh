#!/bin/bash
# Программный сброс USB камеры. Драйвер должен быть остановлен.
set -e
cd "$(dirname "$(readlink -f "$0")")/.."
exec env -u SNAP -u SNAP_LIBRARY_PATH -u PYTHONPATH python3 tools/reset_camera.py "$@"
