// Zero Pose Demo
// This example demonstrates how to move robot to zero pose.
//
// Usage:
//     ./example_zero_pose <server address> [servo]
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

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address> [servo]" << std::endl;
    return 1;
  }

  std::string address{argv[1]};
  std::string servo = kAll;
  if (argc >= 3) {
    servo = argv[2];
  }

  auto robot = Robot<y1_model::A>::Create(address);

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

  constexpr size_t kTorsoDOF = y1_model::A::kTorsoIdx.size();
  constexpr size_t kRightArmDOF = y1_model::A::kRightArmIdx.size();
  constexpr size_t kLeftArmDOF = y1_model::A::kLeftArmIdx.size();

  double minimum_time = 10.0;

  std::cout << "Moving to zero pose..." << std::endl;

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder()
                                             .SetMinimumTime(minimum_time)
                                             .SetPosition(Eigen::VectorXd::Zero(kTorsoDOF)))
                        .SetRightArmCommand(JointPositionCommandBuilder()
                                                .SetMinimumTime(minimum_time)
                                                .SetPosition(Eigen::VectorXd::Zero(kRightArmDOF)))
                        .SetLeftArmCommand(JointPositionCommandBuilder()
                                               .SetMinimumTime(minimum_time)
                                               .SetPosition(Eigen::VectorXd::Zero(kLeftArmDOF))))))
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::cerr << "Error: Failed to move to zero pose." << std::endl;
    return 1;
  }

  std::cout << "Successfully moved to zero pose." << std::endl;
  return 0;
}
