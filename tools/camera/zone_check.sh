#!/bin/bash
# Что попадает в рабочий круг. --save кладёт эталон.
#
# Перед запуском проверяется, что камера в рабочем состоянии: драйвер поднят,
# мёртвой области нет.
# Пропустить проверку: --skip-preflight
set -e
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/../_lib/env.sh"
source "$HERE/../_lib/preflight.sh"

ARGS=(); SKIP=0
for a in "$@"; do
    case "$a" in
        --skip-preflight) SKIP=1 ;;
        *) ARGS+=("$a") ;;
    esac
done

robohand_env ros
cd "$ROBOHAND_ROOT"
if [ "$SKIP" -eq 0 ]; then
    robohand_camera_ready
fi
exec env "${ROBOHAND_ENV[@]}" "$ROBOHAND_PY" tools/camera/zone_check.py "${ARGS[@]}"
