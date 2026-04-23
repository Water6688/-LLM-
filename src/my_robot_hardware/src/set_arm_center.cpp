#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <feetech_driver/common.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

using namespace feetech_driver;

int main(int argc, char* argv[]) {
  const std::string port_name = (argc > 1) ? argv[1] : "/dev/ttyUSB0";

  auto serial_port = std::make_unique<SerialPort>(port_name);
  const auto configure_result = serial_port->configure();
  if (!configure_result) {
    std::cerr << "Failed to open serial port " << port_name << ": " << configure_result.error() << std::endl;
    return 1;
  }

  CommunicationProtocol protocol(std::move(serial_port));

  // Default joint IDs for robot arm
  const std::vector<uint8_t> ids = {1, 2, 3, 4, 5, 6};
  const int center_position = 2048;  // Center position for STS3215
  const int speed = 2400;
  const int acceleration = 50;

  std::cout << "Setting all robot arm joints to center position (2048) on " << port_name << std::endl;

  for (const auto id : ids) {
    if (const auto ping_result = protocol.ping(id); !ping_result) {
      std::cout << "ID " << static_cast<int>(id) << ": ping failed -> " << ping_result.error() << std::endl;
      continue;
    }

    if (const auto write_result = protocol.write_position(id, center_position, speed, acceleration); !write_result) {
      std::cout << "ID " << static_cast<int>(id) << ": position write failed -> " << write_result.error() << std::endl;
      continue;
    }

    std::cout << "ID " << static_cast<int>(id) << ": set to center position" << std::endl;
  }

  std::cout << "All joints set to center position. Waiting for movement to complete..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  std::cout << "Verifying positions:" << std::endl;
  for (const auto id : ids) {
    const auto position_result = protocol.read_position(id);
    if (!position_result) {
      std::cout << "ID " << static_cast<int>(id) << ": position read failed -> " << position_result.error() << std::endl;
      continue;
    }

    const auto current_position = position_result.value();
    const auto angle = to_angle(current_position);
    std::cout << "ID " << static_cast<int>(id) << ": position=" << current_position
              << " angle=" << angle << " deg" << std::endl;
  }

  std::cout << "Robot arm centering complete!" << std::endl;
  return 0;
}