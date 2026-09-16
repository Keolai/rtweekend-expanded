#!/bin/bash

for file in out[0-9]*[0-9]*[0-9]*[0-9]*.ppm; do
    # Skip if no files matched
    [ -e "$file" ] || continue

    # Replace .ppm with .png
    output="converted_recordings/${file%.ppm}.png"

    convert "$file" "$output"
done