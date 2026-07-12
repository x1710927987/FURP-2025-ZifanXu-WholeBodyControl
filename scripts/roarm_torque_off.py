#!/usr/bin/env python3
"""Send RoArm-M3 torque OFF (cmd=0) over serial."""
import sys
import time

from roarm_sdk.roarm import roarm

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
arm = roarm(roarm_type="roarm_m3", port=port, baudrate=115200)
result = arm.torque_set(cmd=0)
print(f"Torque OFF sent on {port}, result={result}")
time.sleep(0.3)
