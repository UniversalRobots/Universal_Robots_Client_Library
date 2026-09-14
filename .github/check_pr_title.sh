#!/usr/bin/env bash
# Check a PR title for meaningfulness (heuristics), spelling (cspell), and
# grammar (LanguageTool). Intended for CI; can also be run locally:
#   PR_TITLE='My title here' .github/check_pr_title.sh
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CSPELL_CONFIG="${SCRIPT_DIR}/cspell-pr-title.json"

TITLE="${PR_TITLE:-${1:-}}"
if [ -z "${TITLE}" ]; then
  echo "Usage: PR_TITLE='...' $0   or   $0 'title'" >&2
  exit 2
fi

FAILED=0

summary() {
  if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
    echo "$1" | tee -a "${GITHUB_STEP_SUMMARY}"
  else
    echo "$1"
  fi
}

summary "## PR title check"
summary ""
summary "Title: \`${TITLE}\`"
summary ""

# Release PRs use Release/X.Y.Z and are exempt from all checks.
if echo "${TITLE}" | grep -Eq '^Release/[0-9]+\.[0-9]+\.[0-9]+$'; then
  summary "Skipped: release title exception (\`Release/X.Y.Z\`)."
  exit 0
fi

# --- Heuristics (meaningfulness) ---
summary "### Heuristics"
if [ -z "${TITLE// }" ]; then
  summary "- Empty or whitespace-only title"
  FAILED=1
fi
if [ "${TITLE}" != "${TITLE#"${TITLE%%[![:space:]]*}"}" ] || \
   [ "${TITLE}" != "${TITLE%"${TITLE##*[![:space:]]}"}" ]; then
  summary "- Leading or trailing whitespace"
  FAILED=1
fi
WORD_COUNT=$(echo "${TITLE}" | wc -w)
if [ "${WORD_COUNT}" -lt 3 ]; then
  summary "- Fewer than 3 words (found ${WORD_COUNT})"
  FAILED=1
fi
TITLE_LOWER=$(echo "${TITLE}" | tr '[:upper:]' '[:lower:]')
case "${TITLE_LOWER}" in
  fix|update|wip|test|changes|misc|tmp|temp)
    summary "- Title is on the denylist (\`${TITLE_LOWER}\`)"
    FAILED=1
    ;;
esac
if [ "${#TITLE}" -lt 12 ]; then
  summary "- Length is less than 12 characters (found ${#TITLE})"
  FAILED=1
fi
if [ "${FAILED}" -eq 0 ]; then
  summary "- OK"
fi
summary ""

# --- Spelling (cspell) ---
summary "### Spelling (cspell)"
if ! command -v cspell >/dev/null 2>&1; then
  npm install --global cspell@8
fi
if ! CSPELL_OUT=$(echo "${TITLE}" | cspell --config "${CSPELL_CONFIG}" --no-progress stdin 2>&1); then
  summary '```'
  summary "${CSPELL_OUT}"
  summary '```'
  FAILED=1
else
  summary "- OK"
fi
summary ""

# --- Grammar (LanguageTool, local Docker) ---
summary "### Grammar (LanguageTool)"
docker rm -f languagetool >/dev/null 2>&1 || true
docker run -d --name languagetool -p 8010:8010 erikvl87/languagetool
cleanup_lt() {
  docker rm -f languagetool >/dev/null 2>&1 || true
}
trap cleanup_lt EXIT

for i in $(seq 1 60); do
  if curl -sf http://localhost:8010/v2/languages >/dev/null; then
    break
  fi
  if [ "${i}" -eq 60 ]; then
    summary "- LanguageTool did not become ready in time"
    exit 1
  fi
  sleep 2
done

DISABLED_RULES='WHITESPACE_RULE,EN_QUOTES,DASH_RULE,WORD_CONTAINS_UNDERSCORE,UPPERCASE_SENTENCE_START,ARROWS,COMMA_PARENTHESIS_WHITESPACE,UNLIKELY_OPENING_PUNCTUATION,SENTENCE_WHITESPACE,CURRENCY,EN_UNPAIRED_BRACKETS,PHRASE_REPETITION,PUNCTUATION_PARAGRAPH_END,METRIC_UNITS_EN_US,ENGLISH_WORD_REPEAT_BEGINNING_RULE'

LT_JSON=$(curl -sf \
  --data-urlencode "text=${TITLE}" \
  --data "language=en-US" \
  --data "disabledRules=${DISABLED_RULES}" \
  --data "disabledCategories=TYPOS" \
  http://localhost:8010/v2/check)

LT_MATCHES=$(echo "${LT_JSON}" | python3 -c '
import json, sys
data = json.load(sys.stdin)
matches = data.get("matches", [])
for m in matches:
    rule = m.get("rule", {}).get("id", "")
    message = m.get("message", "")
    replacements = ", ".join(r.get("value", "") for r in m.get("replacements", [])[:5])
    line = f"- {message} [{rule}]"
    if replacements:
        line += f" (suggestions: {replacements})"
    print(line)
')

if [ -n "${LT_MATCHES}" ]; then
  summary "${LT_MATCHES}"
  FAILED=1
else
  summary "- OK"
fi
summary ""

if [ "${FAILED}" -ne 0 ]; then
  summary "**Result: failed**"
  exit 1
fi
summary "**Result: passed**"
