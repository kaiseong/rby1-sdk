// Dynamics Robot Demo
// This example demonstrates how to use the robot's dynamics model for inverse dynamics calculation.
// It connects to the robot, retrieves the dynamics model, and computes joint torques.
//
// Usage:
//     ./example_30_dynamics_robot <server address>
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

#include <iostream>
#include <random>
#include <vector>
#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <server address>" << std::endl;
    return 1;
  }

  std::string address{argv[1]};

  // 1. Connect to the robot and get its model information.
  auto robot = Robot<y1_model::A>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect to the robot." << std::endl;
    return 1;
  }

  // 2. Get the dynamics model (dyn_robot) from the robot.
  auto dyn_robot = robot->GetDynamics();

  // 3. Create a State object for dynamics calculations.
  //    Specify the names of the links and joints to be used.
  constexpr size_t DOF = y1_model::A::kRobotDOF;
  std::vector<std::string> link_names = {"base", "ee_right"};
  std::vector<std::string> joint_names(y1_model::A::kRobotJointNames.begin(),
                                       y1_model::A::kRobotJointNames.end());

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
  //    Forward kinematics-related calculations must be performed first.
  dyn_robot->ComputeForwardKinematics(dyn_state);
  dyn_robot->ComputeDiffForwardKinematics(dyn_state);
  dyn_robot->Compute2ndDiffForwardKinematics(dyn_state);
  dyn_robot->ComputeInverseDynamics(dyn_state);

  // 6. Print the calculated torques.
  Eigen::IOFormat fmt(4, 0, ", ", "\n", "  [", "]");
  std::cout << "Inverse dynamics torque (Nm):" << std::endl;
  std::cout << dyn_state->GetTau().transpose().format(fmt) << std::endl;

  return 0;
}
