// Multi Controls Demo
// This example demonstrates sending multiple joint position commands with different priorities.
//
// Usage example:
//   ./example_27_multi_controls --address 192.168.30.1:50051 --model a --power ".*" --servo ".*" 
//  
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std;

namespace {

constexpr double kD2R = 0.017453292519943295;

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

template <typename ModelT>
struct Pose {
  Eigen::VectorXd torso;
  Eigen::VectorXd right_arm;
  Eigen::VectorXd left_arm;
};

template <typename ModelT>
Pose<ModelT> ReadyPose() {
  Pose<ModelT> pose;

  pose.torso.resize(ModelT::kTorsoIdx.size());
  pose.right_arm.resize(ModelT::kRightArmIdx.size());
  pose.left_arm.resize(ModelT::kLeftArmIdx.size());

  const double torso_deg[6] = {0.0, 45.0, -90.0, 45.0, 0.0, 0.0};
  const double right_deg[7] = {0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0};
  const double left_deg[7] = {0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0};

  for (size_t i = 0; i < ModelT::kTorsoIdx.size(); ++i) {
    pose.torso(static_cast<Eigen::Index>(i)) = torso_deg[i] * kD2R;
  }
  for (size_t i = 0; i < ModelT::kRightArmIdx.size(); ++i) {
    pose.right_arm(static_cast<Eigen::Index>(i)) = right_deg[i] * kD2R;
  }
  for (size_t i = 0; i < ModelT::kLeftArmIdx.size(); ++i) {
    pose.left_arm(static_cast<Eigen::Index>(i)) = left_deg[i] * kD2R;
  }

  return pose;
}

template <typename ModelT>
std::shared_ptr<Robot<ModelT>> InitializeRobot(const std::string& address, const std::string& power,
                                               const std::string& servo) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot " << address << std::endl;
    return nullptr;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return nullptr;
  }
  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to turn power (" << power << ") on" << std::endl;
      return nullptr;
    }
  }
  if (!robot->IsServoOn(servo)) {
    if (!robot->ServoOn(servo)) {
      std::cerr << "Failed to servo (" << servo << ") on" << std::endl;
      return nullptr;
    }
  }

  auto cms = robot->GetControlManagerState();
  if (cms.state == ControlManagerState::State::kMajorFault || cms.state == ControlManagerState::State::kMinorFault) {
    if (!robot->ResetFaultControlManager()) {
      std::cerr << "Failed to reset control manager" << std::endl;
      return nullptr;
    }
  }
  if (!robot->EnableControlManager()) {
    std::cerr << "Failed to enable control manager" << std::endl;
    return nullptr;
  }

  return robot;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>] [--servo <regex>]"
            << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] [servo_regex]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  auto robot = InitializeRobot<ModelT>(address, power, servo);
  if (!robot) {
    return 1;
  }

  const double minimum_time = 2.0;

  const auto pose = ReadyPose<ModelT>();

  Eigen::VectorXd body_zero(ModelT::kBodyIdx.size());
  body_zero.setZero();

  auto robot_handle = robot->SendCommand(
      RobotCommandBuilder().SetCommand(
          ComponentBasedCommandBuilder().SetBodyCommand(
              JointPositionCommandBuilder().SetPosition(body_zero).SetMinimumTime(minimum_time))),
      1);
  
  std::this_thread::sleep_for(1s);

  auto right_handle = robot->SendCommand(
      RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
          BodyComponentBasedCommandBuilder().SetRightArmCommand(
              JointPositionCommandBuilder().SetPosition(pose.right_arm).SetMinimumTime(minimum_time)))),
      10);

  std::this_thread::sleep_for(1s);

  auto left_handle = robot->SendCommand(
      RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
          BodyComponentBasedCommandBuilder().SetLeftArmCommand(
              JointPositionCommandBuilder().SetPosition(pose.left_arm).SetMinimumTime(minimum_time)))),
      1);

  const auto robot_feedback = robot_handle->Get();
  const auto right_feedback = right_handle->Get();
  const auto left_feedback = left_handle->Get();

  std::cout << "robot_handle.finish_code = " << static_cast<int>(robot_feedback.finish_code()) << std::endl;
  std::cout << "right_handle.finish_code = " << static_cast<int>(right_feedback.finish_code()) << std::endl;
  std::cout << "left_handle.finish_code = " << static_cast<int>(left_feedback.finish_code()) << std::endl;

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string power = ".*";
  std::string servo = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg == "--servo" && i + 1 < argc) {
      servo = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
    } else if (servo == ".*") {
      servo = arg;
    } else {
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (address.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  model = ToLower(model);

  if (model == "a") {
    return Run<y1_model::A>(address, power, servo);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, power, servo);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address, power, servo);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
