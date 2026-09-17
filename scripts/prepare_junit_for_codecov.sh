#!/usr/bin/env bash
# Rewrite CTest --output-junit XML so Codecov Test Analytics will ingest it.
# Codecov looks for *junit.xml and a <testsuites> wrapper; CTest writes
# build/junit.xml with a bare <testsuite name="(empty)">.
set -euo pipefail

usage() {
  echo "usage: $0 <input.xml> <output.junit.xml> [suite-name]" >&2
  exit 2
}

[[ $# -ge 2 ]] || usage
src=$1
dst=$2
suite=${3:-ctest}

[[ -f "$src" ]] || {
  echo "missing junit file: $src" >&2
  exit 1
}
case "$dst" in
  *junit.xml) ;;
  *)
    echo "output must be named *junit.xml (Codecov Test Analytics glob)" >&2
    exit 1
    ;;
esac

mkdir -p "$(dirname "$dst")"
cp "$src" "$dst"

tmp=$(mktemp)
sed "s/name=\"(empty)\"/name=\"${suite}\"/" "$dst" >"$tmp"
mv "$tmp" "$dst"

if grep -q '<testsuites' "$dst"; then
  exit 0
fi

tmp=$(mktemp)
awk -v suite="$suite" '
  NR == 1 {
    if ($0 ~ /<\?xml/) {
      print
      print "<testsuites name=\"" suite "\">"
      next
    }
    print "<testsuites name=\"" suite "\">"
  }
  { print }
  END { print "</testsuites>" }
' "$dst" >"$tmp"
mv "$tmp" "$dst"
