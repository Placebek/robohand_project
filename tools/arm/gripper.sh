#!/bin/bash
# Присоска ES01: on / off / без аргумента — спросить состояние.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/arm/gripper.py "$@"
