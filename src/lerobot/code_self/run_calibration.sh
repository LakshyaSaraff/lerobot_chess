#!/bin/bash
# Ensures we use the correct python interpreter from the environment
# instead of the system python (/bin/python3).

echo "=== DEBUG INFO ==="
which python3
python3 --version
python3 -c "import sys; print(sys.executable)"
python3 -c "import serial; print('serial file:', serial.__file__); print('dir(serial):', dir(serial))"
echo "=================="

python3 test_gripper_load.py
