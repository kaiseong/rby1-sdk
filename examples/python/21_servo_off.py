# Servo Off Example
#
# This example brings up the robot, disables the control manager after a short delay, and turns off the
# servo for the specified joints. See --help for arguments.
# Note: This example uses local helper functions. See helper.py in this file path.
#
# Usage example:
#     python 21_servo_off.py --address 192.168.30.1:50051 --model a --power '.*' --servo '^(head).*'
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import time
import helper
import argparse
import logging

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def main(address, model, power, servo):
    robot = helper.initialize_robot(address, model, power, servo)

    logging.info(
        "Robot was successfully brought up. The control manager will be disabled and the servo will be turned off in "
        "5 second.")
    time.sleep(5)
    robot.disable_control_manager()

    robot.servo_off(servo)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="21_servo_off")
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
        default="^(head).*",
        help="Servo name regex pattern (default: '.*')",
    )
    args = parser.parse_args()

    main(
        address=args.address,
        model=args.model,
        power=args.power,
        servo=args.servo,
    )
