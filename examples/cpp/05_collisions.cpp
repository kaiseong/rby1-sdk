// Collisions Demo
// This example is part of the RB-Y1 SDK examples. See --help for arguments.
//
// Usage example:
//   ./example_05_collisions --address 192.168.30.1:50051 --model a --power ".*"
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
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Dense>
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

std::string FormatEigenVec3(const Eigen::Vector3d& v) {
  Eigen::IOFormat fmt(3, 0, ", ", ", ", "[", "]", "[", "]");
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss << std::setprecision(3) << v.format(fmt);
  return ss.str();
}

std::string CollisionStr(const dyn::CollisionResult& cr) {
  std::ostringstream ss;
  ss << "CollisionResult(link1=" << cr.link1 << ", link2=" << cr.link2
     << ", position1=" << FormatEigenVec3(cr.position1) << ", position2=" << FormatEigenVec3(cr.position2)
     << ", distance=" << std::fixed << std::setprecision(6) << cr.distance << ")";
  return ss.str();
}

void PrintUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --address <server address> [--model a|m|ub] [--power <regex>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [power_regex]" << std::endl;
}

template <typename ModelT>
int RunRobotCollisions(const std::string& address, const std::string& power_regex) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }
  if (!robot->IsConnected()) {
    std::cerr << "Robot is not connected" << std::endl;
    return 1;
  }
  if (!robot->IsPowerOn(power_regex)) {
    if (!robot->PowerOn(power_regex)) {
      std::cerr << "Failed to power on" << std::endl;
      return 1;
    }
  }

  robot->StartStateUpdate(
      [](const auto& state) {
        if (!state.collisions.empty()) {
          const auto& collision = state.collisions.front();
          if (collision.distance < 0) {
            std::cout << ">>>>> Collision detected!" << std::endl;
          }
          std::cout << CollisionStr(collision) << std::endl;
        }
      },
      10.0);

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
    return RunRobotCollisions<y1_model::A>(address, power);
  }
  if (model == "m") {
    return RunRobotCollisions<y1_model::M>(address, power);
  }
  if (model == "ub") {
    return RunRobotCollisions<y1_model::UB>(address, power);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
