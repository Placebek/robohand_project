#!/bin/bash
# Сквозная проверка калибровки: пиксель -> координаты руки.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run ros tools/calib/check.py "$@"
