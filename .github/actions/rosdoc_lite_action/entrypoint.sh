#!/bin/bash

source /opt/ros/noetic/setup.bash

IGNORE_PATTERN_FILE=.rosdoc_ignore_patterns

OUT_DIR=$(mktemp -d)

# We are only interested in stderr, hence we forward stdout to /dev/null
# As we want to parse the output using sed, we need to pipe stderr to stdout afterwards to we can process it
err_output=$(rosdoc_lite . -o ${OUT_DIR} 2>&1 > /dev/null | grep -v -E -f $IGNORE_PATTERN_FILE)

if [[ -z "$err_output" ]]; then
  echo "Check successful. rosdoc_lite didn't report any errors or warnings."
else
  echo $err_output
  echo "rosdoc_lite found errors or warnings."
  exit 1
fi
exit 0
