#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
export MPLCONFIGDIR="$root/runtime/matplotlib"
export XDG_CACHE_HOME="$root/runtime/cache"
exec "$root/.venv/bin/blenderproc" run --custom-blender-path "$root/runtime/blender-4.2.1-linux-x64" --temp-dir /tmp "$root/scripts/generate.py" "$@"
