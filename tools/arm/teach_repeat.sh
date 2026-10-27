#!/bin/bash
# Ручное обучение, запись точек, воспроизведение и обратный цикл.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/arm/teach_repeat.py "$@"
