#!/bin/bash
# Тычок в точку -> нормаль. Без --approach рука не движется.
# --pick: полный цикл захвата — подвод, вакуум, посадка, подъём.
#
# Перед запуском проверяется, что камера в рабочем состоянии: драйвер поднят,
# мёртвой области нет.
# Дополнительно сверяется положение камеры с эталоном — этот инструмент
# опирается на калибровку, а сдвиг камеры делает её молча неверной.
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

robohand_env qt
cd "$ROBOHAND_ROOT"
if [ "$SKIP" -eq 0 ]; then
    robohand_camera_ready --verify-zone
fi
exec env "${ROBOHAND_ENV[@]}" python3 tools/measure/pick_normal.py "${ARGS[@]}"
