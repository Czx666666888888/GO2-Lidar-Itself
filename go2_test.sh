#!/usr/bin/env bash
# Single PC entry; default only prints the plan.
set -euo pipefail
go2_test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$go2_test_root/scripts/go2_full_test.py" "$@"
