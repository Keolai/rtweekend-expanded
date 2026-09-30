#!/bin/bash

OUTPUT_DIR="converted_recordings"

echo "Removing old PNG files..."
rm -f "$OUTPUT_DIR"/out[0-9]*[0-9]*[0-9]*[0-9]*.png

for file in out[0-9]*[0-9]*[0-9]*[0-9]*.ppm; do
    # Skip if no files matched
    [ -e "$file" ] || continue

    # Replace .ppm with .png
    filename=$(basename "$file")
    output="$OUTPUT_DIR/${filename%.ppm}.png"

     if convert "$file" "$output"; then
        rm "$file"
    else
        echo "ERROR: Failed to convert $file — keeping original"
    fi
    
done