# Real Time Control Demo
# This example demonstrates how to control the robot using real time control. See --help for arguments.
#
# Usage example:
#     python 28_real_time_control.py --address 127.0.0.1:50051 --model a
#
# Scenario
# - real-time control cannot use the builder type commands provided by the existing SDK
# - In this example, a separate controller is implemented and used.
# 1. stream on
# 2. Move to ready position
# 3. wait next command
# Copyright (c) 2025 Rainbow Robotics. All rights reserved.
#
# DISCLAIMER:
# This is a sample code provided for educational and reference purposes only.
# Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
# the use or misuse of this demo code. Please use with caution and at your own discretion.

import rby1_sdk as rby
import helper
import numpy as np
import argparse
import logging
import threading

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


class RealTimeControl:
    def __init__(self, address, model, power, servo):
        self.robot = helper.initialize_robot(address, model, power, servo)
        self.model = self.robot.model()

        self.target_position = None
        self.minimum_time = 1

        self.local_t = 0.0
        self.generator = None
        state = self.robot.get_state()
        self.last_target_position = state.position
        self.last_target_velocity = state.velocity
        if model == "a":
            self.rt_thread = threading.Thread(
                target=self.robot.control,
                args=(self.control_a,),
            )
        elif model == "m":
            self.rt_thread = threading.Thread(
                target=self.robot.control,
                args=(self.control_m,),
            )

    def set_target(self, position, minimum_time=1):
        self.target_position = position
        self.minimum_time = minimum_time

    def control_a(self, state: rby.Robot_A_ControlState):
        i = rby.Robot_A_ControlInput()

        if self.target_position is not None:
            if self.generator is None:
                self.generator = rby.math.TrapezoidalMotionGenerator()

            gen_inp = rby.math.TrapezoidalMotionGenerator.Input()
            gen_inp.current_position = self.last_target_position
            gen_inp.current_velocity = self.last_target_velocity
            gen_inp.target_position = self.target_position
            gen_inp.velocity_limit = np.array([5.0] * self.model.robot_dof)
            gen_inp.acceleration_limit = np.array([5.0] * self.model.robot_dof)
            gen_inp.minimum_time = self.minimum_time
            self.generator.update(gen_inp)
            self.local_t = 0.002
            
            self.target_position = None
            
        if self.generator is not None:
            out = self.generator(self.local_t)
            self.last_target_position = out.position
            self.last_target_velocity = out.velocity

        i.target = self.last_target_position
        i.feedback_gain.fill(10)
        i.feedforward_torque.fill(0)
        i.finish = False
        
        self.local_t += 0.002

        return i

    def control_m(self, state: rby.Robot_M_ControlState):
        i = rby.Robot_M_ControlInput()

        if self.target_position is not None:
            if self.generator is None:
                self.generator = rby.math.TrapezoidalMotionGenerator()

            gen_inp = rby.math.TrapezoidalMotionGenerator.Input()
            gen_inp.current_position = self.last_target_position
            gen_inp.current_velocity = self.last_target_velocity
            gen_inp.target_position = self.target_position
            gen_inp.velocity_limit = np.array([5.0] * self.model.robot_dof)
            gen_inp.acceleration_limit = np.array([5.0] * self.model.robot_dof)
            gen_inp.minimum_time = self.minimum_time
            self.generator.update(gen_inp)
            self.local_t = 0.002
            
            self.target_position = None
            
        if self.generator is not None:
            out = self.generator(self.local_t)
            self.last_target_position = out.position
            self.last_target_velocity = out.velocity

        i.target = self.last_target_position
        i.feedback_gain.fill(10)
        i.feedforward_torque.fill(0)
        i.finish = False
        
        self.local_t += 0.002

        return i

    def start(self):
        self.rt_thread.start()

    def wait_for_done(self):
        self.rt_thread.join()


def main(address, model, power, servo):
    rt_control = RealTimeControl(address, model, power, servo)
    rt_control.start()
    if model == "a":
        rt_control.set_target(
            np.deg2rad(
                [
                # wheel
                0.0,0.0,
                # torso
                0.0,45.0,-90.0,45.0,0.0,0.0,
                # right arm
                0.0,-5.0,0.0,-120.0,0.0,70.0,0.0,
                # left arm
                0.0,5.0,0.0,-120.0,0.0,70.0,0.0,
                # head
                0.0,0.0,
            ]
        ),
        minimum_time=2,
    )
    elif model == "m":
        rt_control.set_target(
            np.deg2rad(
                [
                # wheel
                0.0,0.0,0.0,0.0,
                # torso
                0.0,45.0,-90.0,45.0,0.0,0.0,
                # right arm
                0.0,-5.0,0.0,-120.0,0.0,70.0,0.0,
                # left arm
                0.0,5.0,0.0,-120.0,0.0,70.0,0.0,
                # head
                0.0,0.0,
            ]
        ),
        minimum_time=2,
    )

    rt_control.wait_for_done()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="26_joint_group_command")
    parser.add_argument("--address", type=str, required=True, help="Robot address")
    parser.add_argument(
        "--model", 
        type=str, 
        default="a", 
        help="Robot Model Name (default: 'a')"
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

