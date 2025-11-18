#!/bin/bash
# Launcher script for Advanced Transformer Simulator

echo "=================================="
echo "Advanced Transformer Simulator"
echo "=================================="
echo ""

# Check if dependencies are installed
echo "Checking dependencies..."
python3 -c "import numpy, matplotlib, scipy" 2>/dev/null

if [ $? -ne 0 ]; then
    echo "Installing required dependencies..."
    pip3 install -r requirements.txt
fi

echo "Starting simulator..."
echo ""

# Run the simulator
python3 transformer_parallel_simulator.py

echo ""
echo "Simulator closed."
