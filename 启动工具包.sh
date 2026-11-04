#!/usr/bin/env bash
set -euo pipefail
ROBOTCAL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROBOTCAL_ROS_PYTHONPATH="${ROBOTCAL_ROS_PYTHONPATH:-${PYTHONPATH:-}}"
export PYTHONNOUSERSITE=1
unset PYTHONHOME
export PYTHONPATH="$ROBOTCAL_ROOT"
if [[ -x "$ROBOTCAL_ROOT/runtime/python/bin/python3" ]]; then
  ROBOTCAL_PYTHON="$ROBOTCAL_ROOT/runtime/python/bin/python3"
else
  ROBOTCAL_PYTHON="${ROBOTCAL_PYTHON:-python3}"
fi
exec "$ROBOTCAL_PYTHON" "$ROBOTCAL_ROOT/field.py" "$@"
