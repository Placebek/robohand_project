#!/bin/bash
# Ошибка нормали по кадру. Поверхность должна быть ПЛОСКОЙ.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run ros tools/measure/normal_error.py "$@"
