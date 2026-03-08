#!/usr/bin/env python3
"""
Test script to visualize gripper load (torque/current) values.
Use this to find the THRESHOLD for "object grabbed".
"""

import time
import sys
from arm_control import RobotArm

def main():
    try:
        arm = RobotArm()
        print("Connected to arm.")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    print("=" * 60)
    print(" GRIPPER LOAD TEST")
    print("=" * 60)
    print("1. We will enable torque on the gripper.")
    print("2. We will slowly close the gripper.")
    print("3. Watch the LOAD value print out.")
    print("   - Low load (~0-50) = moving freely")
    print("   - High load (>100) = grabbing something or hitting limit")
    print("=" * 60)

    arm.enable()
    
    # Open gripper first
    print("Opening gripper...")
    arm.set_joint("gripper", 1600)
    time.sleep(1.0)

    try:
        while True:
            cmd = input("\nPress ENTER to close slightly (or 'q' to quit): ").strip().lower()
            if cmd == 'q':
                break
            
            # Close by 50 units
            current_pos = arm.get_joint_position("gripper")
            target_pos = current_pos + 100 # Adjust direction if needed (usually higher = closed)
            
            # Check if this is the right direction for your specific servo install!
            # If 1600 is open and 2400 is closed, +100 closes it.
            
            print(f"Moving to {target_pos}...")
            arm.set_joint("gripper", target_pos)
            
            # Monitor load during move
            max_load = 0
            for _ in range(20):
                load = abs(arm.get_joint_load("gripper"))
                pos = arm.get_joint_position("gripper")
                if load > max_load:
                    max_load = load
                print(f"  Pos: {pos:4d} | Load: {load:4d} | Max: {max_load:4d}", end='\r')
                time.sleep(0.05)
            print() # Newline after loop
            
            print(f"  -> Finished step. Max Load was {max_load}")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nRelaxing gripper...")
        arm.disable()
        arm.close()

if __name__ == "__main__":
    main()
