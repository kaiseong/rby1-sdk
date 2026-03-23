// Robot Model Setting Example
// This example demonstrates how to fetch the current robot model (URDF), inspect/edit it,
// upload a new model, and set the model name (applied after reboot).
//
// Usage example:
//   ./example_13_robot_model --address 192.168.30.1:50051 --model a
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

#include "tinyxml2.h"

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
  robot->Connect();
  if (!robot->IsConnected()) {
    std::cerr << "Failed to connect robot" << std::endl;
    return 1;
  }

  const std::string model = robot->GetRobotModel();

  std::cout << "Current robot model: " << std::endl;
  std::cout << model << std::endl;

  // Modify model (example: inspect head_1 effort limit)
  tinyxml2::XMLDocument doc;
  if (doc.Parse(model.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to parse robot model XML" << std::endl;
    return 1;
  }

  auto* root = doc.RootElement();
  if (!root) {
    std::cerr << "Invalid robot model XML (no root element)" << std::endl;
    return 1;
  }

  for (auto* joint = root->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
    const char* name = joint->Attribute("name");
    if (name && std::string(name) == "head_1") {
      auto* limit = joint->FirstChildElement("limit");
      const char* effort = limit ? limit->Attribute("effort") : nullptr;
      std::cout << "joint effort = " << (effort ? effort : "(none)") << std::endl;
      // Example to update:
      // if (limit) limit->SetAttribute("effort", "500");
    }
  }

//   std::cout << "model_name = " << (root->Attribute("name") ? root->Attribute("name") : "") << std::endl;

  // Upload model and save with name 'temp'
//   tinyxml2::XMLPrinter printer;
//   doc.Print(&printer);
//   std::cout << robot->ImportRobotModel("temp", printer.CStr()) << std::endl;

  // Set robot model (applied after reboot)
//   robot->SetParameter("model_name", "\"temp\"");

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
