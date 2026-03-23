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

  constexpr size_t kTorsoDOF = T::kTorsoIdx.size();
  constexpr size_t kRightArmDOF = T::kRightArmIdx.size();
  constexpr size_t kLeftArmDOF = T::kLeftArmIdx.size();

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

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address> [model=a|m] [servo]" << std::endl;
    return 1;
  }
  int extra_start = 2;
  std::string model = "m";
  if (argc >= 3 && (std::string(argv[2]) == "a" || std::string(argv[2]) == "m")) {
    model = argv[2];
    extra_start = 3;
  }
  if (model == "a") return run<y1_model::A>(argc, argv, extra_start);
  return run<y1_model::M>(argc, argv, extra_start);
}
