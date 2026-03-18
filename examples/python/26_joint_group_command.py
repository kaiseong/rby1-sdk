# Note: This example does not run in simulation.
# Joint Group Command Demo
# This example demonstrates how to control the robot using joint group command. See --help for arguments.
#
# Usage example:
#     python 26_joint_group_command.py --address 127.0.0.1:50051 --model a
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
import argparse
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def main(address, model, power, servo):
    robot = helper.initialize_robot(address, model, power, servo)

    minimum_time = 2

    robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(minimum_time)
                    .set_position(np.zeros(7))
                )
                .set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(minimum_time)
                    .set_position(np.zeros(7)) 
                )
                .set_torso_command(
                    rby.JointGroupPositionCommandBuilder()
                    .set_joint_names(["torso_0", "torso_1"])
                    .set_minimum_time(minimum_time)
                    .set_position(np.array([0, 0]))
                )
            )
        ),
        1,
    ).get()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="26_joint_group_command")
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
