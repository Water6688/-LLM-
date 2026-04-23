#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <feetech_driver/common.hpp>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace feetech_driver;

static void print_usage(const char* app_name) {
  std::cout << "Usage: " << app_name << " <serial_port> <position> [id1,id2,id3|start-end|all]" << std::endl;
  std::cout << "Example: " << app_name << " /dev/ttyUSB0 2048 1,2,3" << std::endl;
  std::cout << "         " << app_name << " /dev/ttyUSB0 2048 1-6" << std::endl;
  std::cout << "         " << app_name << " /dev/ttyUSB0 2048 all" << std::endl;
}

static std::vector<uint8_t> parse_ids(const std::string& text) {
  std::vector<uint8_t> ids;
  if (text == "all") {
    for (uint8_t id = 1; id <= 6; ++id) {
      ids.push_back(id);
    }
    return ids;
  }

  std::stringstream ss(text);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (token.empty()) {
      continue;
    }
    const auto dash_pos = token.find('-');
    if (dash_pos != std::string::npos) {
      const auto start = std::stoi(token.substr(0, dash_pos));
      const auto end = std::stoi(token.substr(dash_pos + 1));
      if (start <= 0 || end <= 0 || start > end) {
        continue;
      }
      for (int i = start; i <= end; ++i) {
        ids.push_back(static_cast<uint8_t>(i));
      }
    } else {
      const auto id = std::stoi(token);
      if (id > 0) {
        ids.push_back(static_cast<uint8_t>(id));
      }
    }
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}

int main(int argc, char* argv[]) {
  if (argc < 3) {
    print_usage(argv[0]);
    return 1;
  }

  const std::string port_name = argv[1];
  const int target_position = std::stoi(argv[2]);
  std::vector<uint8_t> ids = {1, 2, 3, 4, 5, 6};
  if (argc >= 4) {
    ids = parse_ids(argv[3]);
  }

  auto serial_port = std::make_unique<SerialPort>(port_name);
  const auto configure_result = serial_port->configure();
  if (!configure_result) {
    std::cerr << "Failed to open serial port " << port_name << ": " << configure_result.error() << std::endl;
    return 1;
  }

  CommunicationProtocol protocol(std::move(serial_port));

  if (ids.empty()) {
    std::cerr << "No valid IDs were provided." << std::endl;
    return 1;
  }

  std::cout << "Setting STS3215 servos on " << port_name << " to position " << target_position << std::endl;

  // Default parameters
  const int speed = 2400;       // Default speed
  const int acceleration = 50;  // Default acceleration

  for (const auto id : ids) {
    if (const auto ping_result = protocol.ping(id); !ping_result) {
      std::cout << "ID " << static_cast<int>(id) << ": ping failed -> " << ping_result.error() << std::endl;
      continue;
    }

    if (const auto write_result = protocol.write_position(id, target_position, speed, acceleration); !write_result) {
      std::cout << "ID " << static_cast<int>(id) << ": position write failed -> " << write_result.error() << std::endl;
      continue;
    }

    std::cout << "ID " << static_cast<int>(id) << ": set to position " << target_position << std::endl;
  }

  // Wait a bit for servos to reach position
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::cout << "Verifying positions..." << std::endl;
  for (const auto id : ids) {
    const auto position_result = protocol.read_position(id);
    if (!position_result) {
      std::cout << "ID " << static_cast<int>(id) << ": position read failed -> " << position_result.error() << std::endl;
      continue;
    }

    const auto current_position = position_result.value();
    const auto angle = to_angle(current_position);
    std::cout << "ID " << static_cast<int>(id) << ": current position=" << current_position
              << " angle=" << angle << " deg" << std::endl;
  }

  return 0;
}