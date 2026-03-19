# Collisions Demo
# This example connects to an RB-Y1 robot, powers on the specified devices,
# subscribes to state updates, and prints detected collision information. See --help for arguments.
#
# Usage example:
#     python 05_collisions.py --address 192.168.30.1:50051 --model a --power '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import time
import argparse


def callback(robot_state):
    if robot_state.collisions:
        collision = robot_state.collisions[0]
        if collision.distance < 0:
            print(">>>>> Collision detected!")
        print(collision)

def main(address, model, power):
    robot = rby.create_robot(address, model)
    
    if not robot.connect():
        print("Robot is not connected")
        exit(1)
    if not robot.is_power_on(power):
        rv = robot.power_on(power)
        if not rv:
            print("Failed to power on")
            exit(1)

    robot.start_state_update(callback, rate=10)  # Hz
    try:
        time.sleep(10)
    except KeyboardInterrupt:
        print("Stopping state update...")
    finally:
        robot.stop_state_update()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="05_collisions")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    parser.add_argument(
        "--power",
        type=str,
        default=".*",
        help="Power device name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model, power=args.power)
