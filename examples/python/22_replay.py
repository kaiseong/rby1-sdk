# Replay Demo
# This example demonstrates how to replay the robot's positions. See --help for arguments.
#
# Usage example:
#     python 22_replay.py --address 127.0.0.1:50051
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import numpy as np
import time
import sys
import argparse


def pre_processing(address, model):
    robot = rby.create_robot(address, model)
    robot.connect()

    if not robot.is_power_on(".*"):
        print("Power is currently OFF. Attempting to power on...")
        if not robot.power_on(".*"):
            print("Error: Failed to power on the robot.")
            sys.exit(1)
        print("Robot powered on successfully.")
    else:
        print("Power is already ON.")

    if not robot.is_servo_on(".*"):
        print("Servo is currently OFF. Attempting to activate servo...")
        if not robot.servo_on(".*"):
            print("Error: Failed to activate servo.")
            sys.exit(1)
        print("Servo activated successfully.")
    else:
        print("Servo is already ON.")

    control_manager_state = robot.get_control_manager_state()

    if (
        control_manager_state.state == rby.ControlManagerState.State.MinorFault
        or control_manager_state.state == rby.ControlManagerState.State.MajorFault
    ):
        if control_manager_state.state == rby.ControlManagerState.State.MajorFault:
            print("Warning: Detected a Major Fault in the Control Manager.")
        else:
            print("Warning: Detected a Minor Fault in the Control Manager.")

        print("Attempting to reset the fault...")
        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")

    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)

    return robot


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="22_replay")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--model", type=str, required=True, help="Robot model")
    args = parser.parse_args()
    robot = pre_processing(args.address, args.model)
    stream = robot.create_command_stream(10)

    model = robot.model()
    body_indices = model.torso_idx + model.right_arm_idx + model.left_arm_idx
    body_joint_names = [model.robot_joint_names[i] for i in body_indices]

    saved_traj = np.load("recorded.npz", allow_pickle=True)["data"]
    for i, traj_step in enumerate(saved_traj):
        # First step: dt = 5, others: dt = 0.1
        dt = 0.1 if i > 0 else 5
        traj_step_body = traj_step[body_indices]
        rc = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(1)
                )
                .set_minimum_time(dt)
                .set_position(traj_step_body)
            )
        )
        rv = stream.send_command(rc)

        time.sleep(dt * 0.95)
