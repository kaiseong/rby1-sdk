#!/usr/bin/env python3
"""Rewrite all even-numbered C++ example files to match odd-numbered style conventions."""
import pathlib

DIR = pathlib.Path(__file__).parent

files = {}

# ============================================================================
# 02_power_command.cpp
# ============================================================================
files["02_power_command.cpp"] = r'''// Power Control Demo
// This example demonstrates how to power on devices.
//
// Usage example:
//   ./example_02_power_command --address 192.168.30.1:50051 --model a --device ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--device <regex>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& device) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  // Power on the device if it's off.
  if (!robot->IsPowerOn(device)) {
    if (!robot->PowerOn(device)) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  // Power off
  // robot->PowerOff(device);

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string device = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
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
    return Run<y1_model::A>(address, device);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, device);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address, device);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 04_get_robot_state.cpp  (matches Python 04_robot_state_stream.py)
# ============================================================================
files["04_get_robot_state.cpp"] = r'''// Robot State Stream Demo
// This example demonstrates how to get robot state via stream.
//
// Usage example:
//   ./example_04_get_robot_state --address 192.168.30.1:50051 --model a --power ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_stop{false};

void SignalHandler(int) { g_stop = true; }

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }
  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  // Start state update stream at 10 Hz (same as Python: rate=10)
  robot->StartStateUpdate(
      [](const RobotState<ModelT>& state, const ControlManagerState& cm) {
        std::cout << "Timestamp: " << state.timestamp.tv_sec << "."
                  << std::setw(9) << std::setfill('0') << state.timestamp.tv_nsec << std::endl;
        std::cout << "  Position: " << state.position.transpose() << std::endl;
        std::cout << "  Velocity: " << state.velocity.transpose() << std::endl;
        std::cout << "Control Manager State: " << rb::to_string(cm.state) << std::endl;
        for (std::size_t i = 0; i < state.joint_states.size(); ++i) {
          std::cout << "  Joint[" << i << "] Temperature: " << state.joint_states[i].temperature << std::endl;
        }
        std::cout << std::endl;
      },
      10);

  // Sleep until interrupted (Ctrl+C)
  auto deadline = std::chrono::steady_clock::now() + 100s;
  while (!g_stop && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(100ms);
  }

  if (g_stop) {
    std::cout << "Stopping state update..." << std::endl;
  }
  robot->StopStateUpdate();

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);

  std::string address;
  std::string model = "a";
  std::string power = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
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
    return Run<y1_model::A>(address, power);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, power);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address, power);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 06_cancel_control.cpp  (matches Python 06_cancel_control.py)
# ============================================================================
files["06_cancel_control.cpp"] = r'''// Cancel Command Demo
// This example demonstrates how to cancel robot control.
//
// Usage example:
//   ./example_06_cancel_control --address 192.168.30.1:50051 --model a --power ".*" --servo ".*"
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

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
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

template <typename ModelT>
void MoveToZeroPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso = Eigen::VectorXd::Zero(ModelT::kTorsoIdx.size());
  Eigen::VectorXd right_arm = Eigen::VectorXd::Zero(ModelT::kRightArmIdx.size());
  Eigen::VectorXd left_arm = Eigen::VectorXd::Zero(ModelT::kLeftArmIdx.size());

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(3.0).SetPosition(torso))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(3.0).SetPosition(right_arm))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(3.0).SetPosition(left_arm)))),
                              90)
                ->Get();
  std::cout << "zero pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::exit(1);
  }
}

template <typename ModelT>
void MoveToPreControlPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso(6);
  torso << 0.0, 0.1, -0.2, 0.1, 0.0, 0.0;
  Eigen::VectorXd right_arm(7);
  right_arm << 0.2, -0.2, 0.0, -1.0, 0.0, 0.7, 0.0;
  Eigen::VectorXd left_arm(7);
  left_arm << 0.2, 0.2, 0.0, -1.0, 0.0, 0.7, 0.0;

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(torso))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(right_arm))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(left_arm)))),
                              90)
                ->Get();
  std::cout << "pre control pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::exit(1);
  }
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

  MoveToZeroPose(robot);

  // Start move_to_pre_control_pose in a separate thread
  std::thread thread([&robot]() { MoveToPreControlPose(robot); });
  std::cout << "move start. minimum time 5 seconds..." << std::endl;
  std::this_thread::sleep_for(2s);
  std::cout << "cancel control" << std::endl;
  robot->CancelControl();
  thread.join();

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
'''

# ============================================================================
# 08_zero_pose.cpp  (matches Python 08_zero_pose.py)
# ============================================================================
files["08_zero_pose.cpp"] = r'''// Zero Pose Demo
// This example demonstrates how to move robot to zero pose.
//
// Usage example:
//   ./example_08_zero_pose --address 192.168.30.1:50051 --model a --power ".*" --servo ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
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

  // Get DOFs from the model (same as Python: len(model.torso_idx), etc.)
  constexpr size_t torso_dof = ModelT::kTorsoIdx.size();
  constexpr size_t right_arm_dof = ModelT::kRightArmIdx.size();
  constexpr size_t left_arm_dof = ModelT::kLeftArmIdx.size();

  double minimum_time = 10.0;

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                                  BodyComponentBasedCommandBuilder()
                                      .SetTorsoCommand(JointPositionCommandBuilder()
                                                           .SetMinimumTime(minimum_time)
                                                           .SetPosition(Eigen::VectorXd::Zero(torso_dof)))
                                      .SetRightArmCommand(JointPositionCommandBuilder()
                                                              .SetMinimumTime(minimum_time)
                                                              .SetPosition(Eigen::VectorXd::Zero(right_arm_dof)))
                                      .SetLeftArmCommand(JointPositionCommandBuilder()
                                                             .SetMinimumTime(minimum_time)
                                                             .SetPosition(Eigen::VectorXd::Zero(left_arm_dof))))),
                              1)
                ->Get();

  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::cerr << "Failed to conduct movej." << std::endl;
    return 1;
  }

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
'''

# ============================================================================
# 10_get_pid_gain.cpp  (matches Python 10_get_pid_gain.py)
# ============================================================================
files["10_get_pid_gain.cpp"] = r'''// Note: This example does not run in simulation.
// Get PID Gain Demo
// This example demonstrates how to get PID gains of the robot.
//
// Usage example:
//   ./example_10_get_pid_gain --address 127.0.0.1:50051 --model a
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

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);

  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(".*")) {
    if (!robot->PowerOn(".*")) {
      std::cerr << "Failed to power on the robot" << std::endl;
      return 1;
    }
  }

  std::this_thread::sleep_for(500ms);  // Waits for all actuators to be fully powered on

  std::cout << ">>> Retrieving PID gains using component names" << std::endl;

  try {
    auto gain_list = robot->GetTorsoPositionPIDGains();
    for (size_t i = 0; i < gain_list.size(); ++i) {
      std::cout << "[torso_" << i << "] P: " << gain_list[i].p_gain
                << ", I: " << gain_list[i].i_gain << ", D: " << gain_list[i].d_gain << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to get torso PID gains: " << e.what() << std::endl;
  }

  try {
    auto gain_list = robot->GetRightArmPositionPIDGains();
    for (size_t i = 0; i < gain_list.size(); ++i) {
      std::cout << "[right_arm_" << i << "] P: " << gain_list[i].p_gain
                << ", I: " << gain_list[i].i_gain << ", D: " << gain_list[i].d_gain << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to get right arm PID gains: " << e.what() << std::endl;
  }

  try {
    auto gain_list = robot->GetLeftArmPositionPIDGains();
    for (size_t i = 0; i < gain_list.size(); ++i) {
      std::cout << "[left_arm_" << i << "] P: " << gain_list[i].p_gain
                << ", I: " << gain_list[i].i_gain << ", D: " << gain_list[i].d_gain << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to get left arm PID gains: " << e.what() << std::endl;
  }

  try {
    auto gain_list = robot->GetHeadPositionPIDGains();
    for (size_t i = 0; i < gain_list.size(); ++i) {
      std::cout << "[head_" << i << "] P: " << gain_list[i].p_gain
                << ", I: " << gain_list[i].i_gain << ", D: " << gain_list[i].d_gain << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to get head PID gains: " << e.what() << std::endl;
  }

  std::cout << ">>> Retrieving PID gains using joint names" << std::endl;

  for (const auto& joint_name : {"torso_0", "right_arm_0", "left_arm_0", "head_0"}) {
    try {
      auto gain = robot->GetPositionPIDGain(joint_name);
      std::cout << "[" << joint_name << "] P: " << gain.p_gain
                << ", I: " << gain.i_gain << ", D: " << gain.d_gain << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Failed to get PID gain for joint '" << joint_name << "': " << e.what() << std::endl;
    }
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
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
    return Run<y1_model::A>(address);
  }
  if (model == "m") {
    return Run<y1_model::M>(address);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 12_fatory_default_pid_gain.cpp  (matches Python 12_factory_default_pid_gain.py)
# ============================================================================
files["12_fatory_default_pid_gain.cpp"] = r'''// Note: This example does not run in simulation.
// Factory Default PID Gain Demo
// This example demonstrates how to set PID gains of the robot to factory default values.
// It is recommended not to use this example.
//
// Usage example:
//   ./example_12_fatory_default_pid_gain --address 127.0.0.1:50051 --model a
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

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
void PrintPIDGains(const std::shared_ptr<Robot<ModelT>>& robot) {
  auto gain_list = robot->GetTorsoPositionPIDGains();
  for (size_t i = 0; i < gain_list.size(); ++i) {
    std::cout << "[torso_" << i << "] p gain: " << gain_list[i].p_gain
              << ", i gain: " << gain_list[i].i_gain << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetRightArmPositionPIDGains();
  for (size_t i = 0; i < gain_list.size(); ++i) {
    std::cout << "[right_arm_" << i << "] p gain: " << gain_list[i].p_gain
              << ", i gain: " << gain_list[i].i_gain << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetLeftArmPositionPIDGains();
  for (size_t i = 0; i < gain_list.size(); ++i) {
    std::cout << "[left_arm_" << i << "] p gain: " << gain_list[i].p_gain
              << ", i gain: " << gain_list[i].i_gain << ", d gain: " << gain_list[i].d_gain << std::endl;
  }

  gain_list = robot->GetHeadPositionPIDGains();
  for (size_t i = 0; i < gain_list.size(); ++i) {
    std::cout << "[head_" << i << "] p gain: " << gain_list[i].p_gain
              << ", i gain: " << gain_list[i].i_gain << ", d gain: " << gain_list[i].d_gain << std::endl;
  }
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);

  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(".*")) {
    if (!robot->PowerOn(".*")) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
    std::this_thread::sleep_for(1s);
  }

  // Get PID Gains
  std::cout << ">>> Before" << std::endl;
  PrintPIDGains(robot);

  // Set PID Gains (default value)
  // Torso joints
  robot->SetPositionPIDGain("torso_0", 100, 20, 900);
  robot->SetPositionPIDGain("torso_1", 1000, 38, 900);
  robot->SetPositionPIDGain("torso_2", 1000, 38, 900);
  robot->SetPositionPIDGain("torso_3", 220, 40, 400);
  robot->SetPositionPIDGain("torso_4", 50, 20, 400);
  robot->SetPositionPIDGain("torso_5", 220, 40, 400);

  // Right arm joints
  robot->SetPositionPIDGain("right_arm_0", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_1", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_2", 80, 15, 200);
  robot->SetPositionPIDGain("right_arm_3", 35, 5, 80);
  robot->SetPositionPIDGain("right_arm_4", 30, 5, 70);
  robot->SetPositionPIDGain("right_arm_5", 30, 5, 70);
  robot->SetPositionPIDGain("right_arm_6", 100, 5, 120);

  // Left arm joints
  robot->SetPositionPIDGain("left_arm_0", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_1", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_2", 80, 15, 200);
  robot->SetPositionPIDGain("left_arm_3", 35, 5, 80);
  robot->SetPositionPIDGain("left_arm_4", 30, 5, 70);
  robot->SetPositionPIDGain("left_arm_5", 30, 5, 70);
  robot->SetPositionPIDGain("left_arm_6", 100, 5, 150);

  // Head joints
  robot->SetPositionPIDGain("head_0", 800, 0, 4000);
  robot->SetPositionPIDGain("head_1", 800, 0, 4000);

  // Ensure PID Gain update completion
  std::this_thread::sleep_for(50ms);

  std::cout << "\n\n>>> After" << std::endl;
  PrintPIDGains(robot);

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
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
    return Run<y1_model::A>(address);
  }
  if (model == "m") {
    return Run<y1_model::M>(address);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 14_led.cpp  (matches Python 14_led.py)
# ============================================================================
files["14_led.cpp"] = r'''// LED Demo
// This example demonstrates how to control the LED of the robot.
//
// Usage example:
//   ./example_14_led --address 127.0.0.1:50051 --model a
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

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  std::cout << "#1 Red 0.5s" << std::endl;
  robot->SetLEDColor(Color(255, 0, 0), 0.5, 0.1, false);
  std::this_thread::sleep_for(500ms);

  std::cout << "#2 Green 0.5s" << std::endl;
  robot->SetLEDColor(Color(0, 255, 0), 0.5, 0.1, false);
  std::this_thread::sleep_for(500ms);

  std::cout << "#3 Blue 0.5s" << std::endl;
  robot->SetLEDColor(Color(0, 0, 255), 0.5, 0.1, false);
  std::this_thread::sleep_for(500ms);

  std::cout << "#4 White Blinking 1s" << std::endl;
  robot->SetLEDColor(Color(200, 200, 200), 1.0, 0.1, true, 4.0);
  std::this_thread::sleep_for(1s);

  // Rainbow colors
  std::vector<Color> rainbow_colors = {
      Color(255, 0, 0),    // Red
      Color(255, 127, 0),  // Orange
      Color(255, 255, 0),  // Yellow
      Color(0, 255, 0),    // Green
      Color(0, 0, 255),    // Blue
      Color(75, 0, 130),   // Indigo
      Color(148, 0, 211),  // Violet
  };

  std::cout << "#5 Rainbow" << std::endl;
  for (const auto& color : rainbow_colors) {
    robot->SetLEDColor(color, 0.5, 0.2, false);
    std::this_thread::sleep_for(500ms);
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
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
    return Run<y1_model::A>(address);
  }
  if (model == "m") {
    return Run<y1_model::M>(address);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 18_check_firmware_version.cpp  (matches Python 18_check_firmware_version.py)
# ============================================================================
files["18_check_firmware_version.cpp"] = r'''// Check Firmware Version Demo
// This example demonstrates how to check the motor info (name, brake, product_name,
// firmware version) of the robot.
//
// Usage example:
//   ./example_18_check_firmware_version --address 127.0.0.1:50051 --model a --power ".*"
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

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(power)) {
    if (!robot->PowerOn(power)) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }
  std::this_thread::sleep_for(500ms);

  auto robot_info = robot->GetRobotInfo();
  for (const auto& ji : robot_info.joint_infos) {
    std::cout << ji.name << ", " << (ji.has_brake ? "True" : "False") << ", "
              << ji.product_name << ", " << ji.firmware_version << std::endl;
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string power = ".*";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--power" && i + 1 < argc) {
      power = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (power == ".*") {
      power = arg;
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
    return Run<y1_model::A>(address, power);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, power);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address, power);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 20_wifi.cpp  (matches Python 20_wifi.py - simplified interactive version)
# ============================================================================
files["20_wifi.cpp"] = r'''// Note: This example does not run in simulation.
// WiFi Setup Demo
// This example demonstrates how to setup the robot's WiFi connection.
// After changing the IP, please check the OLED on the robot's backpack to confirm.
// It may take 1-2 minutes for the change to take effect.
//
// Usage example:
//   ./example_20_wifi --address 127.0.0.1:50051 --model a
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  // 1. Scan WiFi networks
  std::cout << "Scanning Wi-Fi..." << std::endl;
  auto wifi_networks = robot->ScanWifi();
  std::cout << "Scanning completed!" << std::endl;

  if (wifi_networks.empty()) {
    std::cout << "No WiFi networks found." << std::endl;
    return 1;
  }

  // 2. Display available networks (same as Python)
  std::cout << "\nAvailable Wi-Fi networks:" << std::endl;
  for (size_t i = 0; i < wifi_networks.size(); ++i) {
    std::cout << "  " << (i + 1) << ". " << wifi_networks[i].ssid
              << " (Signal: " << wifi_networks[i].signal_strength
              << ", Secured: " << (wifi_networks[i].secured ? "Yes" : "No") << ")" << std::endl;
  }

  // 3. Select a network
  std::cout << "\nSelect a WiFi network (number): ";
  int selection = 0;
  std::cin >> selection;
  std::cin.ignore();

  if (selection < 1 || selection > static_cast<int>(wifi_networks.size())) {
    std::cerr << "Invalid selection." << std::endl;
    return 1;
  }
  const auto& selected = wifi_networks[selection - 1];
  std::cout << "Selected: " << selected.ssid << std::endl;

  // 4. Enter password if secured
  std::string password;
  if (selected.secured) {
    std::cout << "Enter password: ";
    std::getline(std::cin, password);
  }

  // 5. DHCP setting
  std::cout << "Use DHCP? (y/n): ";
  std::string dhcp_input;
  std::getline(std::cin, dhcp_input);
  bool use_dhcp = (ToLower(dhcp_input) == "y");

  std::string ip_address, gateway;
  std::vector<std::string> dns;

  if (!use_dhcp) {
    std::cout << "Enter IP address: ";
    std::getline(std::cin, ip_address);

    std::cout << "Enter Gateway: ";
    std::getline(std::cin, gateway);

    std::cout << "Enter DNS (comma separated): ";
    std::string dns_input;
    std::getline(std::cin, dns_input);
    std::istringstream iss(dns_input);
    std::string item;
    while (std::getline(iss, item, ',')) {
      // Trim whitespace
      auto start = item.find_first_not_of(' ');
      auto end = item.find_last_not_of(' ');
      if (start != std::string::npos) {
        dns.push_back(item.substr(start, end - start + 1));
      }
    }
  }

  // 6. Connect
  std::cout << "Wait for connecting to " << selected.ssid << "..." << std::endl;
  robot->ConnectWifi(selected.ssid, password, use_dhcp, ip_address, gateway, dns);
  std::cout << "Connected to " << selected.ssid << "! Check OLED on the robot's backpack." << std::endl;

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
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
    return Run<y1_model::A>(address);
  }
  if (model == "m") {
    return Run<y1_model::M>(address);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 22_joint_impedance_control.cpp  (matches Python 22_joint_impedance_control.py)
# ============================================================================
files["22_joint_impedance_control.cpp"] = r'''// Joint Impedance Control Demo
// This example demonstrates how to control the robot's joints using joint impedance control.
// Scenario:
//   1. Move to the pre-control pose
//   2. Run joint impedance control for both arms
//   3. Command both arms toward the zero position with stiffness, damping, and torque limits
//
// Usage example:
//   ./example_22_joint_impedance_control --address 127.0.0.1:50051 --model a --power ".*" --servo ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
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

template <typename ModelT>
bool MoveToPreControlPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso(6);
  torso << 0.0, 0.1, -0.2, 0.1, 0.0, 0.0;
  Eigen::VectorXd right_arm(7);
  right_arm << 0.2, -0.2, 0.0, -1.0, 0.0, 0.7, 0.0;
  Eigen::VectorXd left_arm(7);
  left_arm << 0.2, 0.2, 0.0, -1.0, 0.0, 0.7, 0.0;

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(torso))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(right_arm))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(left_arm)))),
                              90)
                ->Get();
  std::cout << "pre control pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    return false;
  }
  return true;
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

  if (!MoveToPreControlPose(robot)) {
    return 1;
  }

  // Joint Impedance Control on BOTH arms (same as Python)
  constexpr size_t right_arm_dof = ModelT::kRightArmIdx.size();
  constexpr size_t left_arm_dof = ModelT::kLeftArmIdx.size();

  auto handler = robot->SendCommand(
      RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
          BodyComponentBasedCommandBuilder()
              .SetRightArmCommand(JointImpedanceControlCommandBuilder()
                                     .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(10.0))
                                     .SetPosition(Eigen::VectorXd::Zero(right_arm_dof))
                                     .SetMinimumTime(5.0)
                                     .SetStiffness(Eigen::VectorXd::Constant(right_arm_dof, 100.0))
                                     .SetDampingRatio(1.0)
                                     .SetTorqueLimit(Eigen::VectorXd::Constant(right_arm_dof, 10.0)))
              .SetLeftArmCommand(JointImpedanceControlCommandBuilder()
                                    .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(10.0))
                                    .SetPosition(Eigen::VectorXd::Zero(left_arm_dof))
                                    .SetMinimumTime(5.0)
                                    .SetStiffness(Eigen::VectorXd::Constant(left_arm_dof, 100.0))
                                    .SetDampingRatio(1.0)
                                    .SetTorqueLimit(Eigen::VectorXd::Constant(left_arm_dof, 10.0))))));

  auto rv = handler->Get();
  std::cout << "Finish Code: " << static_cast<int>(rv.finish_code()) << std::endl;

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
'''

# ============================================================================
# 26_joint_group_command.cpp  (matches Python 26_joint_group_command.py)
# ============================================================================
files["26_joint_group_command.cpp"] = r'''// Note: This example does not run in simulation.
// Joint Group Command Demo
// This example demonstrates how to control the robot using joint group command.
//
// Usage example:
//   ./example_26_joint_group_command --address 127.0.0.1:50051 --model a --power ".*" --servo ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
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

template <typename ModelT>
bool MoveToPreControlPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso(6);
  torso << 0.0, 0.1, -0.2, 0.1, 0.0, 0.0;
  Eigen::VectorXd right_arm(7);
  right_arm << 0.2, -0.2, 0.0, -1.0, 0.0, 0.7, 0.0;
  Eigen::VectorXd left_arm(7);
  left_arm << 0.2, 0.2, 0.0, -1.0, 0.0, 0.7, 0.0;

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(torso))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(right_arm))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(left_arm)))),
                              90)
                ->Get();
  std::cout << "pre control pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    return false;
  }
  return true;
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

  if (!MoveToPreControlPose(robot)) {
    return 1;
  }

  double minimum_time = 2.0;

  // Same as Python: right_arm and left_arm to zero, torso uses JointGroupPositionCommand
  // for torso_1, torso_2, torso_3 with [0.1, -0.2, 0.1]
  Eigen::Vector3d torso_group_pos;
  torso_group_pos << 0.1, -0.2, 0.1;

  auto rv = robot
                ->SendCommand(
                    RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                        BodyComponentBasedCommandBuilder()
                            .SetRightArmCommand(JointPositionCommandBuilder()
                                                    .SetMinimumTime(minimum_time)
                                                    .SetPosition(Eigen::VectorXd::Zero(7)))
                            .SetLeftArmCommand(JointPositionCommandBuilder()
                                                   .SetMinimumTime(minimum_time)
                                                   .SetPosition(Eigen::VectorXd::Zero(7)))
                            .SetTorsoCommand(JointGroupPositionCommandBuilder()
                                                 .SetJointNames({"torso_1", "torso_2", "torso_3"})
                                                 .SetPosition(torso_group_pos)
                                                 .SetMinimumTime(minimum_time)))),
                    1)
                ->Get();
  std::cout << "joint group command finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    return 1;
  }

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
'''

# ============================================================================
# 28_real_time_control_command.cpp  (matches Python 28_real_time_control.py)
# ============================================================================
files["28_real_time_control_command.cpp"] = r'''// Real Time Control Demo
// This example demonstrates how to control the robot using real time control.
//
// Scenario:
//   - Real-time control cannot use the builder type commands provided by the existing SDK.
//   - In this example, a separate controller is implemented and used.
//   1. Move to zero position
//   2. Start real-time control
//   3. Move to target position using TrapezoidalMotionGenerator
//   4. Wait for done (press Ctrl+C to stop)
//
// Usage example:
//   ./example_28_real_time_control_command --address 127.0.0.1:50051 --model a --power ".*" --servo ".*"
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "rby1-sdk/control_manager_state.h"
#include "rby1-sdk/math/trapezoidal_motion_generator.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/robot_command_builder.h"

using namespace rb;
using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

std::atomic<bool> g_stop{false};
void SignalHandler(int) { g_stop = true; }

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
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

template <typename ModelT>
void MoveToZeroPose(const std::shared_ptr<Robot<ModelT>>& robot) {
  Eigen::VectorXd torso = Eigen::VectorXd::Zero(ModelT::kTorsoIdx.size());
  Eigen::VectorXd right_arm = Eigen::VectorXd::Zero(ModelT::kRightArmIdx.size());
  Eigen::VectorXd left_arm = Eigen::VectorXd::Zero(ModelT::kLeftArmIdx.size());

  auto rv = robot
                ->SendCommand(RobotCommandBuilder().SetCommand(ComponentBasedCommandBuilder().SetBodyCommand(
                    BodyComponentBasedCommandBuilder()
                        .SetTorsoCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(torso))
                        .SetRightArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(right_arm))
                        .SetLeftArmCommand(JointPositionCommandBuilder().SetMinimumTime(5.0).SetPosition(left_arm)))),
                              90)
                ->Get();
  std::cout << "pre control pose finish_code: " << static_cast<int>(rv.finish_code()) << std::endl;
  if (rv.finish_code() != RobotCommandFeedback::FinishCode::kOk) {
    std::exit(1);
  }
}

// RealTimeControl class — mirrors the Python RealTimeControl class
template <typename ModelT>
class RealTimeControl {
 public:
  static constexpr size_t kDOF = ModelT::kRobotDOF;

  RealTimeControl(const std::string& address, const std::string& power, const std::string& servo) {
    robot_ = InitializeRobot<ModelT>(address, power, servo);
    if (!robot_) {
      std::exit(1);
    }

    vel_limit_.setConstant(M_PI);
    acc_limit_.setConstant(M_PI);

    // Go to zero position
    MoveToZeroPose(robot_);
  }

  void SetTarget(const Eigen::Vector<double, kDOF>& position, double minimum_time = 1.0) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_position_ = position;
    minimum_time_ = minimum_time;
    has_target_ = true;
  }

  void Start() {
    rt_thread_ = std::thread([this]() {
      robot_->Control([this](const ControlState<ModelT>& state) -> ControlInput<ModelT> {
        return ControlCallback(state);
      });
    });
  }

  void WaitForDone() {
    while (rt_thread_.joinable()) {
      if (g_stop) {
        std::cout << "\nInterrupted! Stopping control stream gracefully..." << std::endl;
        is_running_ = false;
        break;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (rt_thread_.joinable()) {
      rt_thread_.join();
    }
  }

 private:
  ControlInput<ModelT> ControlCallback(const ControlState<ModelT>& state) {
    ControlInput<ModelT> input;

    if (!initialized_) {
      last_target_position_ = state.position;
      last_target_velocity_ = state.velocity;
      initialized_ = true;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_target_) {
        typename TrapezoidalMotionGenerator<kDOF>::Input gen_inp;
        gen_inp.current_position = last_target_position_;
        gen_inp.current_velocity = last_target_velocity_;
        gen_inp.target_position = target_position_;
        gen_inp.velocity_limit = vel_limit_;
        gen_inp.acceleration_limit = acc_limit_;
        gen_inp.minimum_time = minimum_time_;
        generator_.Update(gen_inp);
        local_t_ = 0.002;
        has_target_ = false;
      }
    }

    auto out = generator_(local_t_);
    last_target_position_ = out.position;
    last_target_velocity_ = out.velocity;

    input.target = last_target_position_;
    input.feedback_gain.setConstant(10);
    input.feedforward_torque.setConstant(0);
    input.finish = !is_running_;

    local_t_ += 0.002;
    return input;
  }

  std::shared_ptr<Robot<ModelT>> robot_;
  TrapezoidalMotionGenerator<kDOF> generator_;
  Eigen::Vector<double, kDOF> vel_limit_;
  Eigen::Vector<double, kDOF> acc_limit_;

  std::mutex mutex_;
  Eigen::Vector<double, kDOF> target_position_;
  double minimum_time_{1.0};
  bool has_target_{false};

  Eigen::Vector<double, kDOF> last_target_position_;
  Eigen::Vector<double, kDOF> last_target_velocity_;
  bool initialized_{false};
  double local_t_{0.0};

  std::atomic<bool> is_running_{true};
  std::thread rt_thread_;
};

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>] [--servo <regex>]"
            << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex] [servo_regex]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& power, const std::string& servo) {
  RealTimeControl<ModelT> rt_control(address, power, servo);
  rt_control.Start();

  double robot_minimum_time = 5.0;

  // Target position in degrees, converted to radians (same as Python)
  Eigen::Vector<double, ModelT::kRobotDOF> target;
  target.setZero();

  if constexpr (std::string_view(ModelT::kModelName) == "A") {
    // wheel(2) + torso(6) + right_arm(7) + left_arm(7) + head(2) = 24
    target << 0.0, 0.0,                                  // wheel
        0.0, 45.0, -90.0, 45.0, 0.0, 0.0,                // torso
        0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0,          // right arm
        0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0,           // left arm
        0.0, 0.0;                                          // head
  } else if constexpr (std::string_view(ModelT::kModelName) == "M") {
    // wheel(4) + torso(6) + right_arm(7) + left_arm(7) + head(2) = 26
    target << 0.0, 0.0, 0.0, 0.0,                        // wheel
        0.0, 45.0, -90.0, 45.0, 0.0, 0.0,                // torso
        0.0, -5.0, 0.0, -120.0, 0.0, 50.0, 0.0,          // right arm
        0.0, 5.0, 0.0, -120.0, 0.0, 50.0, 0.0,           // left arm
        0.0, 0.0;                                          // head
  }

  target = target * M_PI / 180.0;

  rt_control.SetTarget(target, robot_minimum_time);
  rt_control.WaitForDone();

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);

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

  std::cerr << "Unknown model: " << model << " (only 'a' and 'm' supported for this example)" << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 30_dynamics_robot.cpp  (matches Python 30_dynamics_robot.py)
# ============================================================================
files["30_dynamics_robot.cpp"] = r'''// Dynamics Robot Demo
// This example demonstrates how to use the robot's dynamics model for inverse dynamics
// calculation.  It connects to the robot, retrieves the dynamics model, and computes
// joint torques.
//
// Usage example:
//   ./example_30_dynamics_robot --address 127.0.0.1:50051 --model a
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address) {
  // 1. Connect to the robot and get its model information.
  auto robot = Robot<ModelT>::Create(address);

  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Failed to connect to the robot." << std::endl;
    return 1;
  }

  // 2. Get the dynamics model (dyn_robot) from the robot.
  auto dyn_robot = robot->GetDynamics();

  // 3. Create a State object for dynamics calculations.
  //    Specify the names of the links and joints to be used in the calculations.
  constexpr size_t DOF = ModelT::kRobotDOF;
  std::vector<std::string> link_names = {"base", "ee_right"};
  std::vector<std::string> joint_names(ModelT::kRobotJointNames.begin(), ModelT::kRobotJointNames.end());
  auto dyn_state = dyn_robot->MakeState(link_names, joint_names);

  // 4. Define and set the robot's state (q, qdot, qddot).
  //    Here, we use random joint angles for demonstration.
  std::mt19937 gen(42);
  std::uniform_real_distribution<double> dist(-0.5, 0.5);
  Eigen::Vector<double, DOF> q;
  for (size_t i = 0; i < DOF; ++i) {
    q(i) = dist(gen) * M_PI / 2.0;
  }

  dyn_state->SetGravity({0, 0, 0, 0, 0, -9.81});
  dyn_state->SetQ(q);
  dyn_state->SetQdot(Eigen::Vector<double, DOF>::Zero());
  dyn_state->SetQddot(Eigen::Vector<double, DOF>::Zero());

  std::cout << "State:" << std::endl;
  std::cout << "  q: " << q.transpose() << std::endl;

  // 5. Perform Inverse Dynamics calculation.
  //    To compute inverse dynamics, forward kinematics-related calculations must be performed first.
  //    These functions cache their results within the dyn_state object.

  // 5-1. Compute forward kinematics (FK) to get the position/orientation of each link.
  dyn_robot->ComputeForwardKinematics(dyn_state);

  // 5-2. Compute the first derivative of FK (Jacobian) to get the velocity of each link.
  //      Must be called after compute_forward_kinematics.
  dyn_robot->ComputeDiffForwardKinematics(dyn_state);

  // 5-3. Compute the second derivative of FK to get the acceleration of each link.
  //      Must be called after compute_diff_forward_kinematics.
  dyn_robot->Compute2ndDiffForwardKinematics(dyn_state);

  // 5-4. Based on the kinematic information calculated above,
  //      compute the joint torques (tau) required to maintain the current state (q, qdot, qddot).
  dyn_robot->ComputeInverseDynamics(dyn_state);

  // 6. Print the calculated torques.
  Eigen::IOFormat fmt(4, 0, ", ", "\n", "  [", "]");
  std::cout << "Inverse dynamics torque (Nm):" << std::endl;
  std::cout << dyn_state->GetTau().transpose().format(fmt) << std::endl;

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
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
    return Run<y1_model::A>(address);
  }
  if (model == "m") {
    return Run<y1_model::M>(address);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# 32_log_stream.cpp  (matches Python 32_log_stream.py)
# ============================================================================
files["32_log_stream.cpp"] = r'''// Log Stream Demo
// This example demonstrates how to get logs from the robot via stream.
//
// Usage example:
//   ./example_32_log_stream --address 127.0.0.1:50051 --model a
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

#include "rby1-sdk/log.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

namespace {

std::string ToLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }

  robot->SyncTime();

  robot->StartLogStream(
      [](const std::vector<Log>& logs) {
        for (const auto& log : logs) {
          if (log.level >= Log::Level::kInfo) {
            std::cout << log << std::endl;
          }
        }
      },
      1.0);

  std::this_thread::sleep_for(60s);

  robot->StopLogStream();

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
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
    return Run<y1_model::A>(address);
  }
  if (model == "m") {
    return Run<y1_model::M>(address);
  }
  if (model == "ub") {
    return Run<y1_model::UB>(address);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
'''

# ============================================================================
# Write all files
# ============================================================================
for name, content in files.items():
    path = DIR / name
    content = content.strip() + "\n"
    path.write_text(content)
    print(f"Written: {path}")

print(f"\nTotal: {len(files)} files written.")
