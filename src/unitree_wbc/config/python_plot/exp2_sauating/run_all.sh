#!/bin/bash

echo "Starting the execution of exp3-toq-leg-proc.py"
python3 exp2-toq-leg-proc.py
echo "Completed the execution of exp3-toq-leg-proc.py"

echo "Starting the execution of exp3-toq-arm-proc.py"
python3 exp2-toq-arm-proc.py
echo "Completed the execution of exp3-toq-arm-proc.py"

echo "Starting the execution of exp3-pos-leg-proc.py"
python3 exp2-pos-leg-proc.py
echo "Completed the execution of exp3-pos-leg-proc.py"

echo "Starting the execution of exp3-pos-arm-proc.py"
python3 exp2-pos-arm-proc.py
echo "Completed the execution of exp3-pos-arm-proc.py"

echo "All scripts executed successfully!"
