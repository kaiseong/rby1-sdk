# Battery Config Demo
# This example demonstrates how to control(set,reset) the battery configuration of the robot. See --help for arguments.
#
# Usage example:
#     python 16_battery_config.py --address 127.0.0.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import argparse
import time


def main(address, model):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        print("Failed to connect robot")
        exit(1)

    print("# Set battery level as 50")
    print(f" -- {'SUCCESS' if robot.set_battery_level(50) else 'FAIL'}")

    time.sleep(1)

    print("# Reset battery configuration")
    print(f" -- {'SUCCESS' if robot.reset_battery_config() else 'FAIL'}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="16_battery_config")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
