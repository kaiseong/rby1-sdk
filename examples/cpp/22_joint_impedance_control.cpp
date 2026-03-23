// Joint Impedance Control Demo
// This example demonstrates how to control the robot's joints using impedance control.
// Scenario:
//   1. Move to zero position
//   2. Move to ready position
//   3. Move only the right arm to zero position with impedance control
//
// Usage:
//     ./example_22_joint_impedance_control <server address> [model=a|m] [servo]
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <chrono>
#include <iostream>
#include <thread>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

const std::string kAll = ".*";

template <typename T>
int run(int argc, char** argv, int extra_start) {
  std::string address{argv[1]};
  std::string servo = kAll;
  if (argc > extra_start) {
    servo = argv[extra_start];
  }

  auto robot = Robot<T>::Create(address);

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  if (!robot->IsPowerOn(kAll)) {
    if (!robot->PowerOn(kAll)) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
  }

  if (!robot->IsServoOn(servo)) {
    if (!robot->ServoOn(servo)) {
      std::cerr << "Error: Failed to activate servo." << std::endl;
      return 1;
    }
  }

  const auto& control_manager_state = robot->GetControlManagerState();
  if (control_manager_state.state == ControlManagerState::State::kMajorFault ||
      control_manager_state.state == ControlManagerState::State::kMinorFault) {
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Error: Unable to reset the fault in the Control Manager." << std::endl;
      return 1;
    }
  }

  if (!robot->EnableControlManager()) {
    std::cerr << "Error: Failed to enable the Control Manager." << std::endl;
    return 1;
  }

  std::this_thread::sleep_for(1s);

  // Step 1: Move to zero position
  std::cout << "Moving to zero position..." << std::endl;
  {
    auto rv =
        robot
            ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                BodyComponentBasedCommandBuilder()
                    .SetTorsoCommand(
                        JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(Eigen::VectorXd::Zero(6)))
                    .SetRightArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(Eigen::VectorXd::Zero(7)))
                    .SetLeftArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(Eigen::VectorXd::Zero(7))))))
            ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to move to zero position." << std::endl;
      return 1;
    }
  }
  std::cout << "Reached zero position." << std::endl;

  // Step 2: Move to ready position
  // torso  : [0, 45, -90, 45, 0, 0] deg
  // right  : [0, -5,   0, -120, 0,  70, 0] deg
  // left   : [0,  5,   0, -120, 0,  70, 0] deg
  std::cout << "Moving to ready position..." << std::endl;
  {
    const double D2R = M_PI / 180.0;

    Eigen::Vector<double, 6> q_torso;
    q_torso << 0, 45, -90, 45, 0, 0;
    q_torso *= D2R;

    Eigen::Vector<double, 7> q_right_arm;
    q_right_arm << 0, -5, 0, -120, 0, 70, 0;
    q_right_arm *= D2R;

    Eigen::Vector<double, 7> q_left_arm;
    q_left_arm << 0, 5, 0, -120, 0, 70, 0;
    q_left_arm *= D2R;

    auto rv =
        robot
            ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                BodyComponentBasedCommandBuilder()
                    .SetTorsoCommand(
                        JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(q_torso))
                    .SetRightArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(q_right_arm))
                    .SetLeftArmCommand(
                        JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(q_left_arm)))))
            ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Failed to move to ready position." << std::endl;
      return 1;
    }
  }
  std::cout << "Reached ready position." << std::endl;

  // Step 3: Joint impedance control — move right arm to zero with impedance
  std::cout << "Executing joint impedance control on right arm..." << std::endl;
  {
    auto rv =
        robot
            ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                BodyComponentBasedCommandBuilder().SetRightArmCommand(
                    JointImpedanceControlCommandBuilder()
                        .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(10.0))
                        .SetPosition(Eigen::VectorXd::Zero(7))
                        .SetMinimumTime(5.0)
                        .SetStiffness(Eigen::VectorXd::Constant(7, 100.0))
                        .SetDampingRatio(1.0)
                        .SetTorqueLimit(Eigen::VectorXd::Constant(7, 10.0))))))
            ->Get();

    if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
      std::cerr << "Error: Joint impedance control failed." << std::endl;
      return 1;
    }
  }
  std::cout << "Joint impedance control completed successfully." << std::endl;

  return 0;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address> [model=a|m] [servo]" << std::endl;
    return 1;
  }
  int extra_start = 2;
  std::string model = "a";
  if (argc >= 3 && (std::string(argv[2]) == "a" || std::string(argv[2]) == "m")) {
    model = argv[2];
    extra_start = 3;
  }
  if (model == "a") return run<y1_model::A>(argc, argv, extra_start);
  return run<y1_model::M>(argc, argv, extra_start);
}
