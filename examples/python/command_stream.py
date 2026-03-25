################### CAUTION ###################
# CAUTION:
# Ensure that the robot has enough surrounding clearance before running this example.
###############################################

# Command Stream Demo
# This example brings up the robot and streams body joint position commands while the last body joint follows a sinusoidal target. See --help for arguments.
#
# Usage example:
#     python command_stream.py --address 192.168.30.1:50051 --model a --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import argparse
import logging
import math
import time
import numpy as np
import rby1_sdk as rby

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

def move_to_zero_pose(robot):
    """ Move to Zero Position Before Starting the Motion """
    torso = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    right_arm = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    left_arm = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_torso_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(5.0)
                    .set_position(torso)
                )
                .set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(5.0)
                    .set_position(right_arm)
                )
                .set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(5.0)
                    .set_position(left_arm)
                )
            )
        ),
        90,
    ).get()
    print(f"pre control pose finish_code: {rv.finish_code}")
    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        exit(1)


def main(address, model, power, servo):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        logging.error(f"Failed to connect robot {address}")
        exit(1)
    if not robot.is_power_on(power):
        if not robot.power_on(power):
            logging.error(f"Failed to turn power ({power}) on")
            exit(1)
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            logging.error(f"Failed to servo ({servo}) on")
            exit(1)
    if robot.get_control_manager_state().state in [
        rby.ControlManagerState.State.MajorFault,
        rby.ControlManagerState.State.MinorFault,
    ]:
        if not robot.reset_fault_control_manager():
            logging.error("Failed to reset control manager")
            exit(1)
    if not robot.enable_control_manager():
        logging.error("Failed to enable control manager")
        exit(1)

    logging.info("===== Command Stream Example =====")

    logging.info(robot.set_parameter("joint_position_command.cutoff_frequency", "5"))
    logging.info(robot.set_parameter("default.acceleration_limit_scaling", "0.8"))

    
    move_to_zero_pose(robot)
    stream = robot.create_command_stream(10)

    dt = 0.001
    for t in range(0, 10000):
        q = [0.0] * 6 + [0.0] * 7 + [0.0] * 6 + [math.pi / 4.0 * math.sin(math.pi * 2 * t * dt / 5)]
        rc = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(1)
                )
                .set_minimum_time(dt)
                .set_position(q)
            )
        )
        stream.send_command(rc)
        time.sleep(dt * 0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="command_stream")
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
