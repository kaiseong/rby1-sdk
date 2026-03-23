// LED Demo
// This example demonstrates how to control the LED of the robot.
//
// Usage:
//     ./example_led <server address>
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
#include <vector>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

template <typename T>
int run(int argc, char** argv) {
  std::string address{argv[1]};
  auto robot = Robot<T>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot" << std::endl;
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

  std::cout << "Rainbow colors..." << std::endl;
  for (const auto& color : rainbow_colors) {
    robot->SetLEDColor(color, 0.5, 0.1, false);
    std::this_thread::sleep_for(500ms);
  }

  std::cout << "Done." << std::endl;
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
