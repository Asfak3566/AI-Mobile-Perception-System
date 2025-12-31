#!/bin/bash

# Check if best.pt exists
if [ ! -f "best.pt" ]; then
    echo "Error: best.pt not found in current directory."
    exit 1
fi

echo "Exporting best.pt to TensorRT engine..."
echo "NOTE: This must be run on the target device (Orin) to ensure compatibility."

# Export using ultralytics CLI
# device=0 uses the first GPU
yolo export model=best.pt format=engine device=0 half=True

if [ $? -eq 0 ]; then
    echo "Export successful! Created best.engine"
else
    echo "Export failed. Ensure 'ultralytics' is installed and CUDA is available."
    echo "Try installing: pip install ultralytics"
fi
