################### CAUTION ###################
# CAUTION:
# This example sends a direct joint position command to the robot body.
# Ensure that the surrounding workspace is clear before running this example.
###############################################

# Joint Position Test
# This example connects to an RB-Y1 robot and sends a joint position command using the currently configured torso and arm poses. See --help for arguments.
#
# Usage example:
#     python test.py --address 192.168.30.1:50051 --model a --power ' .*' --servo ' .*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import argparse

import numpy as np
import rby1_sdk as rby

TORSO_TARGET = np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0])
RIGHT_ARM_TARGET = np.deg2rad([0.0, -5.0, 0.0, -20.0, 0.0, 20.0, 0.0])
LEFT_ARM_TARGET = np.deg2rad([0.0, 5.0, 0.0, -20.0, 0.0, 20.0, 0.0])


def main(address, model, power, servo):
    robot = rby.create_robot(address, model)

    if not robot.connect():
        print(f"Error: Unable to establish connection to the robot at {address}")
        raise SystemExit(1)

    if not robot.is_power_on(power):
        if not robot.power_on(power):
            print("Failed to power on")
            raise SystemExit(1)

    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            print("Failed to servo on")
            raise SystemExit(1)

    control_manager_state = robot.get_control_manager_state()
    if control_manager_state.state in (
        rby.ControlManagerState.State.MinorFault,
        rby.ControlManagerState.State.MajorFault,
    ):
        if not robot.reset_fault_control_manager():
            print("Error: Unable to reset the fault in the Control Manager.")
            raise SystemExit(1)

    if not robot.enable_control_manager():
        print("Error: Failed to enable the Control Manager.")
        raise SystemExit(1)

    command = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(5.0)
            .set_position(
                np.concatenate((TORSO_TARGET, RIGHT_ARM_TARGET, LEFT_ARM_TARGET))
            )
        )
    )

    rv = robot.send_command(command, 10).get()
    print(f"finish_code: {rv.finish_code}")

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="test")
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
