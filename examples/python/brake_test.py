# Brake Test Example
# This example connects to the robot, powers it on if needed, releases the brake
# for a target joint, waits briefly, and then engages the brake again.
#
# Usage example:
#   python brake_test.py --address 192.168.30.1:50051 --joint right_arm_0
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import time
import rby1_sdk as rby
import argparse


def main(address, joint):
    robot = rby.create_robot_a(address)

    if not robot.connect():
        print("Robot is not connected")
        exit(1)

    if not robot.is_power_on(".*"):
        if not robot.power_on(".*"):
            print("Failed to power on")
            exit(1)

    time.sleep(0.5)
    print("Brake Release!")
    if not robot.break_release(joint):
        print("Error: Failed to brake release.")

    time.sleep(0.5)
    print("Brake Engage!")
    if not robot.break_engage(joint):
        print("Error: Failed to brake engage.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="brake_test")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--joint", type=str, required=True, help="Joint name regex pattern"
    )

    args = parser.parse_args()

    if "torso" in args.joint or args.joint == ".*":
        print(f"Warning: Using {args.joint} may cause the robot to collapse.")
    else:
        main(address=args.address, joint=args.joint)
