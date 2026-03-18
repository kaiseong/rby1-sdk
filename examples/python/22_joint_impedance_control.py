# Joint Impedance Control Demo
# This example demonstrates how to control the robot's joints using impedance control. See --help for arguments.
# Scenario
# 1. Move to zero position
# 2. Move to ready position
# 3. Move only the right arm to zero position while impedance control
# Usage example:
#     python 22_joint_impedance_control.py --address 127.0.0.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import helper
import argparse
import numpy as np
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


READY_POSE = {
    "torso": np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
    "right_arm": np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    "left_arm": np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
}


def main(address, model, power, servo):
    robot = helper.initialize_robot(address, model, power, servo)

    model = robot.model()
    helper.movej(  # Zero Pose
        robot,
        np.zeros(len(model.torso_idx)),
        np.zeros(len(model.right_arm_idx)),
        np.zeros(len(model.left_arm_idx)),
        minimum_time=5,
    )
    helper.movej(  # Ready Pose
        robot,
        READY_POSE["torso"],
        READY_POSE["right_arm"],
        READY_POSE["left_arm"],
        minimum_time=5,
    )

    # Joint Impedance Control
    rc_builder = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.BodyComponentBasedCommandBuilder()
            # -- Torso --
            # .set_torso_command(
            #     rby.JointImpedanceControlCommandBuilder()
            #     .set_command_header(
            #         rby.CommandHeaderBuilder().set_control_hold_time(10)
            #     )
            #     .set_position([0.0] * len(model.torso_idx))
            #     # .set_velocity_limit([np.inf] * len(model.torso_idx))
            #     # .set_acceleration_limit([np.inf] * len(model.torso_idx))
            #     .set_minimum_time(5)
            #     .set_stiffness([100.0] * len(model.torso_idx))
            #     .set_damping_ratio(1.0)
            #     # .set_torque_limit([100] * len(model.torso_idx))
            # )
            .set_right_arm_command(
                rby.JointImpedanceControlCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(10)
                )
                .set_position([0.0] * len(model.right_arm_idx))
                .set_minimum_time(5)
                .set_stiffness([100.0] * len(model.right_arm_idx))
                .set_damping_ratio(1.0)
                .set_torque_limit([10] * len(model.right_arm_idx))
            )
            # -- Left Arm --
            # .set_left_arm_command(
            #     rby.JointImpedanceControlCommandBuilder()
            #     .set_command_header(
            #         rby.CommandHeaderBuilder().set_control_hold_time(10)
            #     )
            #     .set_position([0.0] * len(model.left_arm_idx))
            #     # .set_velocity_limit([np.inf] * len(model.torso_idx))
            #     # .set_acceleration_limit([np.inf] * len(model.torso_idx))
            #     .set_minimum_time(5)
            #     .set_stiffness([100.0] * len(model.torso_idx))
            #     .set_damping_ratio(1.0)
            #     # .set_torque_limit([100] * len(model.torso_idx))
            # )
        )
    )
    handler = robot.send_command(rc_builder)
    rv = handler.get()
    logging.info(f"Finish Code: {rv.finish_code}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="22_joint_impedance_control")
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
