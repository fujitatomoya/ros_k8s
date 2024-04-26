#!/bin/bash

### enable debug
#set -exvfC
set -e

# Directory to process
DIR="$1"

# Ensure the directory is provided as an argument
if [ -z "$DIR" ]; then
  echo "Usage: $0 <directory>"
  exit 1
fi

# Check if the provided directory exists
if [ ! -d "$DIR" ]; then
  echo "Error: Directory '$DIR' does not exist."
  exit 1
fi

# Find all symbolic links in the directory (not recursively)
find "$DIR" -maxdepth 1 -type l | while read -r symlink; do
  # Get the actual file to which the symbolic link points
  target_file=$(readlink -f "$symlink")

  # If the target file doesn't exist, warn and skip
  if [ ! -e "$target_file" ]; then
    echo "Warning: Symbolic link '$symlink' points to a non-existent file."
    continue
  fi

  # Copy the content of the original file to the location of the symbolic link
  cp "$target_file" "$symlink" && \
    echo "Replaced symbolic link '$symlink' with the actual file." || \
    echo "Error: Failed to replace symbolic link '$symlink'."
done
