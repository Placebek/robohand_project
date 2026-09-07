#!/bin/bash
# Решение AX=XB по снятому набору. Робот и камера не нужны.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/calib/solve.py "$@"
