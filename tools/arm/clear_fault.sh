#!/bin/bash
# Снять аварию с руки. Ничего не двигает, питание не подаёт.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/arm/clear_fault.py "$@"
