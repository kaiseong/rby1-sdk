// Serial Device Demo
// This example demonstrates how to connect to an RB-Y1 robot and list serial devices.
//
// Usage example:
//   ./example_23_serial_device --address 192.168.30.1:50051
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <iostream>
#include <string>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " --address <server address>" << std::endl;
    return 1;
  }

  std::string address;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (address.empty()) {
      address = arg;
    } else {
      std::cerr << "Usage: " << argv[0] << " --address <server address>" << std::endl;
      return 1;
    }
  }

  if (address.empty()) {
    std::cerr << "Usage: " << argv[0] << " --address <server address>" << std::endl;
    return 1;
  }

  auto robot = Robot<y1_model::A>::Create(address);
  robot->Connect();

  if (!robot->IsConnected()) {
    std::cerr << "Error: Robot connection failed." << std::endl;
    return 1;
  }

  const auto serial_devices = robot->GetSerialDeviceList();
  for (size_t i = 0; i < serial_devices.size(); ++i) {
    std::cout << "[" << i << "] " << serial_devices[i] << std::endl;
  }

  return 0;
}
