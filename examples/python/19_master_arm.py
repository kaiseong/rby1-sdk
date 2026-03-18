# Master Arm Example
#
# This example powers on the UPC master arm, initializes it with its URDF model, and runs a
# gravity-compensated current-control loop while printing the master arm state. See --help for arguments.
# Note: This example is not supported in simulation.
#
# Usage example:
#     python 19_master_arm.py --address 192.168.30.1:50051 --model a
#
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.
#
# Run this example on a UPC to which the master arm is connected.

import os
import rby1_sdk as rby
import numpy as np
import argparse
import time
import datetime
import signal


def main(address, model):
    robot = rby.create_robot(address, model)
    
    if not robot.connect():
        print("Error: Robot connection failed.")
        exit(1)

    if not robot.power_on("12v"):
        print("Error: Failed to power on 12V.")
        exit(1)

    def handler(signum, frame):
        robot.power_off("12v")
        exit(1)

    signal.signal(signal.SIGINT, handler)

    rby.upc.initialize_device(rby.upc.MasterArmDeviceName)

    master_arm_model = f"{os.path.dirname(os.path.realpath(__file__))}/../../models/master_arm/model.urdf"
    master_arm = rby.upc.MasterArm(rby.upc.MasterArmDeviceName)
    master_arm.set_model_path(master_arm_model)
    master_arm.set_control_period(0.01)
    active_ids = master_arm.initialize(verbose=True)
    if len(active_ids) != rby.upc.MasterArm.DeviceCount:
        print("Error: Mismatch in the number of devices detected for RBY Master Arm.")
        exit(1)

    def control(state: rby.upc.MasterArm.State):
        with np.printoptions(suppress=True, precision=3, linewidth=300):
            print(f"--- {datetime.datetime.now().time()} ---")
            print(f"q: {state.q_joint}")
            print(f"g: {state.gravity_term}")
            print(
                f"right: {state.button_right.button}, left: {state.button_left.button}"
            )

        control_input = rby.upc.MasterArm.ControlInput()

        control_input.target_operating_mode.fill(rby.DynamixelBus.CurrentControlMode)
        control_input.target_torque = state.gravity_term

        return control_input

    master_arm.start_control(control)

    time.sleep(100)

    master_arm.stop_control()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="19_master_arm")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", type=str, default="a", help="Robot Model Name (default: 'a')"
    )
    args = parser.parse_args()

    main(address=args.address, model=args.model)
