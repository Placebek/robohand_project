#!/bin/bash
# Что попадает в рабочий круг. --save кладёт эталон.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run ros tools/camera/zone_check.py "$@"
