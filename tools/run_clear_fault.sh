#!/bin/bash
# Сброс аварии на руке. Ничего не двигает, питание не подаёт.
set -e
cd "$(dirname "$(readlink -f "$0")")/.."
exec env -u SNAP -u SNAP_LIBRARY_PATH -u PYTHONPATH python3 tools/clear_arm_fault.py "$@"
