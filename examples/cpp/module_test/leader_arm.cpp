#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/leader_arm.h"

using namespace rb;
using namespace std::chrono_literals;

std::shared_ptr<upc::LeaderArm> leader_arm = std::make_shared<upc::LeaderArm>("/dev/rby1_leader_arm");
std::function<void()> g_robot_power_off;

void signalHandler(int signum) {

  leader_arm->StopControl();
  if (g_robot_power_off) g_robot_power_off();
  exit(signum);
}

template <typename T>
int run(int argc, char** argv) {
  std::string address{argv[1]};
  signal(SIGINT, signalHandler);
  auto robot = Robot<T>::Create(address);
  g_robot_power_off = [robot]() { robot->PowerOff("12v"); };

  std::cout << "Attempting to connect to the robot..." << std::endl;
  if (!robot->Connect()) {
    std::cerr << "Error: Unable to establish connection to the robot at " << address << std::endl;
    return 1;
  }
  std::cout << "Successfully connected to the robot." << std::endl;

  std::cout << "Checking power status..." << std::endl;
  if (!robot->IsPowerOn("12v")) {
    std::cout << "Power is currently OFF. Attempting to power on..." << std::endl;
    if (!robot->PowerOn("12v")) {
      std::cerr << "Error: Failed to power on the robot." << std::endl;
      return 1;
    }
    std::cout << "Robot powered on successfully." << std::endl;
  } else {
    std::cout << "Power is already ON." << std::endl;
  }

  try {
    // Latency timer setting
    upc::InitializeDevice("/dev/rby1_leader_arm");
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  leader_arm->SetModelPath(MODELS_PATH "/leader_arm/model.urdf");
  leader_arm->SetControlPeriod(0.01);  // 100Hz

  auto active_ids = leader_arm->Initialize();
  if (active_ids.size() != upc::LeaderArm::kDeivceCount) {
    return 1;
  }

  bool init = false;
  Eigen::Vector<double, upc::LeaderArm::kDOF / 2> q_right, q_left;
  q_right.setZero();
  q_left.setZero();
  leader_arm->StartControl([&](const upc::LeaderArm::State& state) {
    upc::LeaderArm::ControlInput input;

    if (!init) {
      q_right = state.q_joint(Eigen::seq(0, 6));
      q_left = state.q_joint(Eigen::seq(7, 13));
      init = true;
    }

    if (state.button_right.button == 1) {
      input.target_operating_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(0, 6)) = state.gravity_term(Eigen::seq(0, 6));
      q_right = state.q_joint(Eigen::seq(0, 6));
    } else {
      input.target_operating_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(0, 6)) = q_right;
    }

    if (state.button_left.button == 1) {
      input.target_operating_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(Eigen::seq(7, 13)) = state.gravity_term(Eigen::seq(7, 13));
      q_left = state.q_joint(Eigen::seq(7, 13));
    } else {
      input.target_operating_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(7, 13)) = q_left;
    }
    std::cout << "button : [" << state.button_right.button << ", " << state.button_left.button << "]" << std::endl;
    std::cout << "trigger : [" << state.button_right.trigger << ", " << state.button_left.trigger << "]" << std::endl;
    return input;
  });

  std::this_thread::sleep_for(20s);

  leader_arm->StopControl();
  robot->PowerOff("12v");

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