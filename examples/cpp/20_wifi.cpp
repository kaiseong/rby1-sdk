// Note: This example does not run in simulation.
// WiFi Setup Demo
// This example demonstrates how to setup the robot's WiFi connection.
//
// Usage:
//     ./example_wifi <server address> scan
//     ./example_wifi <server address> connect <ssid> [password] [--no-dhcp --ip <ip> --gateway <gw> --dns <dns>]
//     ./example_wifi <server address> status
//     ./example_wifi <server address> disconnect
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
#include <string>
#include <vector>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

using namespace rb;

void print_usage(const char* prog) {
  std::cerr << "Usage:" << std::endl;
  std::cerr << "  " << prog << " <address> scan" << std::endl;
  std::cerr << "  " << prog << " <address> connect <ssid> [password] [--no-dhcp --ip <ip> --gateway <gw> --dns <dns>]"
            << std::endl;
  std::cerr << "  " << prog << " <address> status" << std::endl;
  std::cerr << "  " << prog << " <address> disconnect" << std::endl;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    print_usage(argv[0]);
    return 1;
  }

  std::string address{argv[1]};
  std::string command{argv[2]};

  auto robot = Robot<y1_model::A>::Create(address);

  if (!robot->Connect()) {
    std::cerr << "Failed to connect robot" << std::endl;
    return 1;
  }

  if (command == "scan") {
    auto networks = robot->ScanWifi();
    std::cout << "Found " << networks.size() << " network(s):" << std::endl;
    for (size_t i = 0; i < networks.size(); ++i) {
      std::cout << "  [" << i << "] " << networks[i].ssid << " (Signal: " << networks[i].signal_strength
                << ", Secured: " << (networks[i].secured ? "Yes" : "No") << ")" << std::endl;
    }

  } else if (command == "connect") {
    if (argc < 4) {
      std::cerr << "Error: SSID is required for connect command." << std::endl;
      print_usage(argv[0]);
      return 1;
    }

    std::string ssid{argv[3]};
    std::string password;
    bool use_dhcp = true;
    std::string ip_address;
    std::string gateway;
    std::vector<std::string> dns;

    int i = 4;
    // First non-flag argument after ssid is password
    if (i < argc && std::string(argv[i]).substr(0, 2) != "--") {
      password = argv[i++];
    }

    while (i < argc) {
      std::string arg{argv[i]};
      if (arg == "--no-dhcp") {
        use_dhcp = false;
      } else if (arg == "--ip" && i + 1 < argc) {
        ip_address = argv[++i];
      } else if (arg == "--gateway" && i + 1 < argc) {
        gateway = argv[++i];
      } else if (arg == "--dns" && i + 1 < argc) {
        dns.push_back(argv[++i]);
      }
      ++i;
    }

    std::cout << "Connecting to WiFi: " << ssid << std::endl;
    bool rv = robot->ConnectWifi(ssid, password, use_dhcp, ip_address, gateway, dns);
    std::cout << (rv ? "SUCCESS" : "FAIL") << std::endl;

  } else if (command == "status") {
    auto status = robot->GetWifiStatus();
    if (status.has_value()) {
      std::cout << "WiFi Status:" << std::endl;
      std::cout << "  SSID:      " << status->ssid << std::endl;
      std::cout << "  Connected: " << (status->connected ? "Yes" : "No") << std::endl;
      std::cout << "  IP:        " << status->ip_address << std::endl;
      std::cout << "  Gateway:   " << status->gateway << std::endl;
      std::cout << "  DNS:       ";
      for (const auto& d : status->dns) {
        std::cout << d << " ";
      }
      std::cout << std::endl;
    } else {
      std::cout << "No WiFi status available." << std::endl;
    }

  } else if (command == "disconnect") {
    bool rv = robot->DisconnectWifi();
    std::cout << (rv ? "SUCCESS" : "FAIL") << std::endl;

  } else {
    std::cerr << "Unknown command: " << command << std::endl;
    print_usage(argv[0]);
    return 1;
  }

  return 0;
}
