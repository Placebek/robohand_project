#!/bin/bash
# Парковка руки перед уходом. РУКА ПОЕДЕТ.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/arm/park.py "$@"
