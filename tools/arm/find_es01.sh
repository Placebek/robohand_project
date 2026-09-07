#!/bin/bash
# Перебор выходов в поисках линии захвата. Рука не движется.
set -e
source "$(dirname "$(readlink -f "$0")")/../_lib/env.sh"
robohand_run plain tools/arm/find_es01.py "$@"
