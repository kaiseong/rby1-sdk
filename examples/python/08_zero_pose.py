# Zero Pose Demo
# This example demonstrates how to move robot to zero pose. See --help for arguments.
#
# Usage example:
#     python 08_zero_pose.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import helper # local helper functions. See helper.py in this file path
import argparse
import numpy as np
import logging

# initialize logger for helper functions
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def main(address, model, power, servo):
    robot = helper.initialize_robot(address, model, power, servo)

    model = robot.model()
    torso_dof = len(model.torso_idx)
    right_arm_dof = len(model.right_arm_idx)
    left_arm_dof = len(model.left_arm_idx)
    helper.movej(
        robot,
        np.zeros(torso_dof),
        np.zeros(right_arm_dof),
        np.zeros(left_arm_dof),
        minimum_time=10,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="08_zero_pose")
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
    parser.add_argument(
        "--servo",
        type=str,
        default=".*",
        help="Servo name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(
        address=args.address,
        model=args.model,
        power=args.power,
        servo=args.servo,
    )
