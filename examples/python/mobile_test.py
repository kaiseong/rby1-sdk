################### CAUTION ###################
# CAUTION:
# Mobile base commands move the robot in the surrounding workspace.
# Ensure that the floor area is clear before running this example.
###############################################

# Mobile Command Demo
# This example brings up the robot and runs several mobility commands using joint velocity and SE(2) velocity control. See --help for arguments.
#
# Usage example:
#     python mobile_test.py --address 192.168.30.1:50051 --model m --power '.*' --servo '.*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import numpy as np
import sys
import time
import argparse
import re
import rby1_sdk as rby

# Note: Y-axis SE(2) commands are supported only on the M model.


D2R = np.pi / 180  # Degree to Radian conversion factor
MINIMUM_TIME = 2.5
LINEAR_VELOCITY_LIMIT = 1.5
ANGULAR_VELOCITY_LIMIT = np.pi * 1.5
ACCELERATION_LIMIT = 1.0
STOP_ORIENTATION_TRACKING_ERROR = 1e-5
STOP_POSITION_TRACKING_ERROR = 1e-5
WEIGHT = 0.0015
STOP_COST = 1e-2
VELOCITY_TRACKING_GAIN = 0.01
MIN_DELTA_COST = 1e-4
PATIENCE = 10


def example_ready_command(robot):
    print("example ready")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_waist = [0, 45 * D2R, -90 * D2R, 45 * D2R, 0, 0]
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(2))
            .set_minimum_time(7.0)
            .set_position(q_joint_waist + q_joint_right_arm + q_joint_left_arm)
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_forward_command(robot):
    print("example_forward_command")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_waist = [0, 45 * D2R, -90 * D2R, 45 * D2R, 0, 0]
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.JointVelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([np.pi] * 4)  # joint velocity
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_backward_command(robot):
    print("example_backward_command")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_waist = [0, 45 * D2R, -90 * D2R, 45 * D2R, 0, 0]
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.JointVelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(MINIMUM_TIME)
            .set_velocity([-np.pi] * 4)  # joint velocity
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_SE2_x_backward_command(robot):
    print("example_SE2_backward_command")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_waist = [0, 45 * D2R, -90 * D2R, 45 * D2R, 0, 0]
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(3)
            .set_velocity([-0.2, 0], 0)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_SE2_x_forward_command(robot):
    print("example_SE2_backward_command")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_waist = [0, 45 * D2R, -90 * D2R, 45 * D2R, 0, 0]
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(1)
            .set_velocity([0.5, 0], 0)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_SE2_y_backward_command(robot):
    print("example_SE2_backward_command")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_waist = [0, 45 * D2R, -90 * D2R, 45 * D2R, 0, 0]
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(1)
            .set_velocity([0, -0.5], 0)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def example_SE2_y_forward_command(robot):
    print("example_SE2_backward_command")

    # Initialize joint positions
    q_joint_waist = np.zeros(6)
    q_joint_right_arm = np.zeros(7)
    q_joint_left_arm = np.zeros(7)

    # Set specific joint positions
    q_joint_waist = [0, 45 * D2R, -90 * D2R, 45 * D2R, 0, 0]
    q_joint_right_arm = [0, -5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]
    q_joint_left_arm = [0, 5 * D2R, 0, -120 * D2R, 0, 70 * D2R, 0]

    rc = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1.0))
            .set_minimum_time(3)
            .set_velocity([0, 0.2], 0)  # linear velocity[m/s], angualr velocity[rad/s]
        )
    )

    rv = robot.send_command(rc, 10).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print("Error: Failed to conduct demo motion.")
        return 1

    return 0


def main(address, model, power, servo):
    print("Attempting to connect to the robot...")
    robot = rby.create_robot(address, model)
    robot.connect()

    if not robot.connect():
        print("Error: Unable to establish connection to the robot at")
        sys.exit(1)

    print("Successfully connected to the robot")

    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)

    if not robot.is_power_on(power):
        rv = robot.power_on(power)
        if not rv:
            print("Failed to power on")
            exit(1)

    if not robot.is_servo_on(servo):
        rv = robot.servo_on(servo)
        if not rv:
            print("Fail to servo on")
            exit(1)

    control_manager_state = robot.get_control_manager_state()

    if (
        control_manager_state.state == rby.ControlManagerState.State.MinorFault
        or control_manager_state.state == rby.ControlManagerState.State.MajorFault
    ):

        if control_manager_state.state == rby.ControlManagerState.State.MajorFault:
            print(
                "Warning: Detected a Major Fault in the Control Manager!!!!!!!!!!!!!!!."
            )
        else:
            print(
                "Warning: Detected a Minor Fault in the Control Manager@@@@@@@@@@@@@@@@."
            )

        print("Attempting to reset the fault...")
        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            sys.exit(1)
        print("Fault reset successfully.")

    print("Control Manager state is normal. No faults detected.")

    print("Enabling the Control Manager...")
    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        sys.exit(1)
    print("Control Manager enabled successfully.")

    # example_ready_command(robot)
    # example_backward_command(robot)
    # example_forward_command(robot)

    example_SE2_y_forward_command(robot)
    example_SE2_y_backward_command(robot)
    example_SE2_x_forward_command(robot)
    example_SE2_x_backward_command(robot)

    # if not example_forward_command(robot):
    #     print("finish motion")
    # if not example_backward_command(robot):
    #     print("finish motion")
    # if not example_SE2_backward_command(robot):
    #     print("finish motion")
    # if not example_turn_left_command(robot):
    #     print("finish motion")
    # if not example_turn_right_command(robot):
    #     print("finish motion")
    print("end of demo")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="mobile_test")
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
