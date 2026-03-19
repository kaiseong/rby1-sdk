# Serial Device List Example
#
# This example connects to the robot and prints the list of available serial devices detected by the system. See --help for arguments.
#
# Usage example:
#     python 23_serial_device.py --address 192.168.30.1:50051
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import argparse


def main(address):
    robot = rby.create_robot_a(address)

    if not robot.connect():
        print("Error: Robot connection failed.")
        exit(1)

    serial_devices = robot.get_serial_device_list()
    for [i, device] in enumerate(serial_devices):
        print(f"[{i}] {device}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="23_serial_device")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    args = parser.parse_args()

    main(address=args.address)
