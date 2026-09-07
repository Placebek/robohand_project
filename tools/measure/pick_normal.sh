#!/bin/bash
# Тычок в точку -> нормаль. Без --approach рука не движется.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run qt tools/measure/pick_normal.py "$@"
