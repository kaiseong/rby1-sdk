# Multi-Controls Example
# This example initializes the robot, sends overlapping body and arm joint position
# commands with different priorities, and prints each command handle's finish code. See --help for arguments.
# Note: This example uses local helper functions. See helper.py in this file path.
#
# Usage example:
#   python 27_multi_controls.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import helper
import numpy as np
import time
import argparse
import logging
from dataclasses import dataclass

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


@dataclass
class Pose:
    right_arm: np.typing.NDArray
    left_arm: np.typing.NDArray


READY_POSE = Pose(
    right_arm=np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    left_arm=np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
)


def main(address, model, power, servo):
    robot: rby.Robot_A = helper.initialize_robot(address, model, power, servo)
    robot_model: rby.Model_A = robot.model()
    minimum_time = 2
    robot_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.JointPositionCommandBuilder()
                .set_position(np.zeros(len(robot_model.body_idx)))
                .set_minimum_time(minimum_time)
            )
        ),
        priority=1,
    )
    time.sleep(1)

    right_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(READY_POSE.right_arm)
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        priority=10,
    )
    time.sleep(1)

    left_handle = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(READY_POSE.left_arm)
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        priority=1,
    )

    print(f"{robot_handle.get().finish_code = }")
    print(f"{right_handle.get().finish_code = }")
    print(f"{left_handle.get().finish_code = }")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="27_multi_controls")
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
