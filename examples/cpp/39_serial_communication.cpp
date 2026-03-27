// Serial Communication Demo
// This example demonstrates how to setup serial communication with the robot.
// See --help for arguments.
//
// Usage example:
//     ./example_39_serial_communication --address 192.168.30.1:50051 --model a --device /dev/ttyUSB1 --baudrate 19200
//
// Default device_path: /dev/ttyUSB1, default baudrate: 19200
//
// Copyright (c) 2025 Rainbow Robotics. All rights reserved.
//
// DISCLAIMER:
// This is a sample code provided for educational and reference purposes only.
// Rainbow Robotics shall not be held liable for any damages or malfunctions resulting from
// the use or misuse of this demo code. Please use with caution and at your own discretion.

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
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
  std::cerr << "Usage: " << prog
            << " --address <server address> [--model a|m] [--device <path>] [--baudrate <rate>]" << std::endl;
  std::cerr << "   or: " << prog << " <server address> [model] [device_path] [baudrate]" << std::endl;
}

template <typename ModelT>
int Run(const std::string& address, const std::string& device_path, int baudrate) {
  auto robot = Robot<ModelT>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Error: Robot connection failed." << std::endl;
    return 1;
  }

  auto dev = robot->OpenSerialStream(device_path, baudrate, 8, 'N', 1);
  if (!dev->Connect(true)) {
    std::cerr << "Failed to connect serial device" << std::endl;
    return 1;
  }

  std::cout << "Listening for incoming data..." << std::endl;

  dev->SetReadCallback([](const std::string& data) {
    std::cout << "<< ";
    for (unsigned char ch : data) {
      std::cout << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ch);
    }
    std::cout << std::dec << std::endl;
  });

  std::string line;
  while (true) {
    std::cout << "\n>> Enter hex data to send (e.g., 'B7B8010401C5C6'), or type 'exit': ";
    if (!std::getline(std::cin, line)) {
      break;
    }

    // Trim whitespace
    size_t start = line.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
      continue;
    }
    line = line.substr(start, line.find_last_not_of(" \t\r\n") - start + 1);

    if (line == "exit") {
      break;
    }

    if (line.size() % 2 != 0) {
      std::cerr << "Hex string must have even number of characters." << std::endl;
      continue;
    }

    // Parse hex string to bytes
    std::vector<char> bytes;
    bool valid = true;
    for (size_t i = 0; i < line.size(); i += 2) {
      unsigned int byte_val;
      std::istringstream iss(line.substr(i, 2));
      if (!(iss >> std::hex >> byte_val)) {
        valid = false;
        break;
      }
      bytes.push_back(static_cast<char>(byte_val));
    }

    if (!valid) {
      std::cerr << "Invalid hex input. Please enter valid hexadecimal characters (0-9, A-F)." << std::endl;
      continue;
    }

    dev->Write(bytes.data(), static_cast<int>(bytes.size()));
  }

  std::cout << "Disconnecting serial stream..." << std::endl;
  dev->Disconnect();
  std::cout << "Disconnected. Exiting." << std::endl;

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string address;
  std::string model = "a";
  std::string device_path = "/dev/ttyUSB1";
  int baudrate = 19200;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--address" && i + 1 < argc) {
      address = argv[++i];
    } else if (arg == "--model" && i + 1 < argc) {
      model = argv[++i];
    } else if (arg == "--device" && i + 1 < argc) {
      device_path = argv[++i];
    } else if (arg == "--baudrate" && i + 1 < argc) {
      baudrate = std::stoi(argv[++i]);
    } else if (arg.rfind("--", 0) == 0) {
      PrintUsage(argv[0]);
      return 1;
    } else if (address.empty()) {
      address = arg;
    } else if (model == "a") {
      model = arg;
    } else if (device_path == "/dev/ttyUSB1") {
      device_path = arg;
    } else {
      baudrate = std::stoi(arg);
    }
  }

  if (address.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  model = ToLower(model);

  if (model == "a") {
    return Run<y1_model::A>(address, device_path, baudrate);
  }
  if (model == "m") {
    return Run<y1_model::M>(address, device_path, baudrate);
  }

  std::cerr << "Unknown model: " << model << std::endl;
  PrintUsage(argv[0]);
  return 1;
}
