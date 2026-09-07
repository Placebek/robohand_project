#!/bin/bash
# ПЕРВОЕ ДВИЖЕНИЕ. РУКА ПОЕДЕТ, человек у аварийного стопа обязателен.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/arm/first_motion.py "$@"
