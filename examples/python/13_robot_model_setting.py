# Robot Model Setting Example
#
# This example includes the following features:
# 1. Loading the current robot model (in URDF format) from the robot.
# 2. Saving a custom robot model to the robot with a specified name.
# 3. Assigning the robot model name for the robot to use (applied after reboot). See --help for arguments.
#
# Usage example:
#     python 13_robot_model_setting.py --address 192.168.30.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.


import rby1_sdk as rby
import argparse
import xml.etree.ElementTree as ET


def main(address, model):
    robot = rby.create_robot(address, model)

    if not robot.connect():
        print("Failed to connect robot")
        exit(1)
    robot_model_name = robot.get_robot_model()
    target_joint_name = "head_1"
    updated_effort = "500"

    print("Current robot model: ")
    print(robot_model_name)

    # Modify model
    model_tree = ET.ElementTree(ET.fromstring(robot_model_name))
    model_root = model_tree.getroot()
    previous_effort = None
    for joint in model_root.findall("joint"):
        if joint.get("name") == target_joint_name:
            limit = joint.find("limit")
            previous_effort = limit.get("effort")
            limit.set("effort", updated_effort)
            break

    print(f"Previous {target_joint_name} effort limit: {previous_effort}")
    print(f"Updated {target_joint_name} effort limit: {updated_effort}")

    # Upload model and save model with name 'temp'
    if robot.import_robot_model("temp", ET.tostring(model_root).decode()):   
        set_model_result = robot.set_parameter("model_name", '"temp"')
        print(f"Set model_name result: {set_model_result}")
    
    # After reboot, the robot will use uploaded robot model


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="13_robot_model")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
