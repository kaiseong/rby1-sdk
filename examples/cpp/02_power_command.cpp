#include <iostream>
#include <thread>
#include <chrono>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;
using namespace std::chrono_literals;

template <typename T>
int run(int argc, char** argv) {
  std::string address{argv[1]};
  auto robot = Robot<T>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Connection failed" << std::endl;
    return 1;
  }

  robot->PowerOn(".*");

  std::this_thread::sleep_for(1s);

  robot->PowerOff(".*");

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