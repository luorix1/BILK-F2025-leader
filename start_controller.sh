#!/bin/bash
# start_controller.sh - Start BILK Controller
# Run with: bash start_controller.sh

set -euo pipefail

echo "BILK Controller - Starting"
echo "=============================="

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 not found"
    exit 1
fi

# Check dependencies
if [ -f "requirements.txt" ]; then
    echo "Installing dependencies..."
    pip3 install -r requirements.txt
fi

# Check file exists
if [ ! -f "controller.py" ]; then
    echo "❌ controller.py not found"
    exit 1
fi

# Run
echo "Starting controller..."
python3 controller.py