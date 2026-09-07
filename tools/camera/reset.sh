#!/bin/bash
# Программный сброс USB камеры. Драйвер должен быть остановлен.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/camera/reset.py "$@"
