#!/bin/bash

folder_path="build/examples"

# Check if the folder exists
if [ ! -d "$folder_path" ]; then
  echo "Folder '$folder_path' not found."
  exit 1
fi

# Iterate over all executables in the folder
for file in $folder_path/*; do
echo $file
# Check if the file is executable
if [[ -f "$file" && -x "$file" ]]; then
    # Execute the file
    ./"$file" $@
    # Check the exit status
    exit_status=$?
    if [[ $exit_status -ne 0 ]]; then
    echo "Execution of '$file' failed with exit status $exit_status."
    exit 1
    fi
    # Delay for 10 seconds to avoid too fast reconnects
    echo "Sleep 10"
    sleep 10
fi
done
