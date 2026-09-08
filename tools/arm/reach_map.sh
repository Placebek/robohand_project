#!/bin/bash
# Какая часть рабочего круга камеры достижима рукой. Ничего не двигает.
# Запускать сразу после решения hand-eye, до измерений.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/arm/reach_map.py "$@"
