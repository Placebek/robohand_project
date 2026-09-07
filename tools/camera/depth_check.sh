#!/bin/bash
# Гейт по глубине (RQ2).
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run qt tools/camera/depth_check.py "$@"
