// Check Firmware Version Demo
// This example demonstrates how to check the motor info (name, brake, product_name, firmware version) of the robot.
//
// Usage:
//     ./example_check_firmware_version <server address>
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

using namespace rb;
using namespace std::chrono_literals;

const std::string kAll = ".*";

template <typename T>
int run(int argc, char** argv) {
  std::string address{argv[1]};
  auto robot = Robot<T>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot" << std::endl;
    return 1;
  }

  if (!robot->IsPowerOn(kAll)) {
    if (!robot->PowerOn(kAll)) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  std::this_thread::sleep_for(500ms);

  auto robot_info = robot->GetRobotInfo();
  for (const auto& ji : robot_info.joint_infos) {
    std::cout << ji.name << ", " << (ji.has_brake ? "true" : "false") << ", " << ji.product_name << ", "
              << ji.firmware_version << std::endl;
  }

  return 0;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address> [model=a|m]" << std::endl;
    return 1;
  }
  std::string model = (argc >= 3 && (std::string(argv[2]) == "a" || std::string(argv[2]) == "m")) ? argv[2] : "m";
  if (model == "a") return run<y1_model::A>(argc, argv);
  return run<y1_model::M>(argc, argv);
}
