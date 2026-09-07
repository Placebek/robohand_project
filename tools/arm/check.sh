#!/bin/bash
# Связь с рукой: порты, режим, поза. Ничего не двигает.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/arm/check.py "$@"
