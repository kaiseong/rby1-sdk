import time
import rby1_sdk
from rby1_sdk import *
import argparse
import numpy as np


def main(address, joint):
    robot = rby1_sdk.create_robot_a(address)
    robot.connect()

    if not robot.is_connected():
        print("Robot is not connected")
        exit(1)

    if not robot.is_power_on(".*"):
        rv = robot.power_on(".*")
        if not rv:
            print("Failed to power on")
            exit(1)

    time.sleep(0.5)
    print("Brake Release!")
    if not robot.brake_release(joint):
        print("Error: Failed to brake release.")

    time.sleep(0.5)
    print("Brake Engage!")
    if not robot.brake_engage(joint):
        print("Error: Failed to brake engage.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="brake test")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument("--joint", type=str, help="Joint name regex pattern")

    args = parser.parse_args()

    if "torso" in args.joint or args.joint == ".*":
        print(f"Warning: Using {args.joint} may cause the robot to collapse.")
    else:
        main(address=args.address, joint=args.joint)
