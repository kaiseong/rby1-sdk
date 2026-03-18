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
import numpy as np

RIGHT_ARM_SERVO = "right_arm_.*"
RIGHT_ARM_COLLISION_POSE = np.deg2rad([45.0, 0.0, 0.0, -150.0, 0.0, 0.0, 0.0])


def callback(robot_state):
    if robot_state.collisions:
        collision = robot_state.collisions[0]
        if collision.distance < 0:
            print(">>>>> Collision detected!")
        print(collision)


def send_right_arm_collision_demo_command(robot, minimum_time=3.0):
    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_position(RIGHT_ARM_COLLISION_POSE)
                    .set_minimum_time(minimum_time)
                )
            )
        ),
        1,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        print(f"Failed to send right-arm collision demo command: {rv.finish_code}")
        return False

    return True



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

    if not robot.is_servo_on(RIGHT_ARM_SERVO):
        if not robot.servo_on(RIGHT_ARM_SERVO):
            print("Failed to servo on the right arm")
            exit(1)

    control_manager_state = robot.get_control_manager_state().state
    if control_manager_state in [
        rby.ControlManagerState.State.MajorFault,
        rby.ControlManagerState.State.MinorFault,
    ]:
        if not robot.reset_fault_control_manager():
            print("Failed to reset control manager")
            exit(1)

    if robot.get_control_manager_state().state != rby.ControlManagerState.State.Enabled:
        if not robot.enable_control_manager():
            print("Failed to enable control manager")
            exit(1)

    robot.start_state_update(callback, rate=10)  # Hz
    try:
        time.sleep(1)
        if not send_right_arm_collision_demo_command(robot):
            exit(1)
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
