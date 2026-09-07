#!/bin/bash
# Просмотр RGB-D, клик по глубине меряет расстояние.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run qt tools/camera/rgbd.py "$@"
