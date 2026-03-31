#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <server address> <joint name>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};
  std::string joint_name{argv[2]};

  if (joint_name.find("torso") != std::string::npos || joint_name == ".*") {
    std::cerr << "Warning: Using '" << joint_name << "' may cause the robot to collapse." << std::endl;
    return 1;
  }

  auto robot = Robot<y1_model::A>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }

  robot->PowerOn(".*");
  std::this_thread::sleep_for(0.5s);
  std::cout << "Brake Release!" << std::endl;
  if (!robot->BrakeRelease(joint_name)) {
    std::cerr << "Error: Failed to brake release." << std::endl;
    return 1;
  }
  std::this_thread::sleep_for(0.5s);
  std::cout << "Brake Engage!" << std::endl;
  robot->BrakeEngage(joint_name);
  if (!robot->BrakeEngage(joint_name)) {
    std::cerr << "Error: Failed to brake engage." << std::endl;
    return 1;
  }

  return 0;
}