#!/bin/bash
# Таблица «дистанция -> пятно, маркер, зона».
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run ros tools/camera/plan.py "$@"
