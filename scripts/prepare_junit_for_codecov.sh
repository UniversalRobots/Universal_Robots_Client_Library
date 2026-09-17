#!/usr/bin/env bash
# Rewrite CTest/GTest JUnit into the Codecov Test Analytics JUnit shape.
set -euo pipefail

dir=$(cd "$(dirname "$0")" && pwd)
if command -v python3 >/dev/null 2>&1; then
  py=python3
elif command -v python >/dev/null 2>&1; then
  py=python
else
  echo "python3 is required to rewrite JUnit for Codecov" >&2
  exit 1
fi
exec "$py" "$dir/prepare_junit_for_codecov.py" "$@"
