#!/bin/bash
# Съёмка поз для hand-eye. РУКА ЗАПИТЫВАЕТСЯ И СТАНОВИТСЯ ПОДАТЛИВОЙ.
#
# Перед съёмкой обязательная подготовка: камера поднимается с чистого листа,
# с руки снимается авария. Оба отказа молчаливые, и наткнуться на любой из них
# посреди серии — потерять все снятые позы. Пропустить: --skip-preflight.
set -e
HERE="$(dirname "$(readlink -f "$0")")"
source "$HERE/../_lib/env.sh"
source "$HERE/../_lib/preflight.sh"

ARGS=()
SKIP=0
for a in "$@"; do
    case "$a" in
        --skip-preflight) SKIP=1 ;;
        *) ARGS+=("$a") ;;
    esac
done

robohand_env qt
cd "$ROBOHAND_ROOT"

if [ "$SKIP" -eq 0 ]; then
    echo "=================== ПОДГОТОВКА ==================="
    robohand_preflight всё
    echo "=========== подготовка закончена ================="
    echo
else
    echo "--skip-preflight: камеру и руку не готовлю"
fi

exec env "${ROBOHAND_ENV[@]}" "$ROBOHAND_PY" tools/calib/capture.py "${ARGS[@]}"
