#pragma once

#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <memory>
#include <array>
#include <vector>
#include <cstdint>

#if __has_include(<hardware_interface/hardware_interface/version.h>)
#include <hardware_interface/hardware_interface/version.h>
#else
#include <hardware_interface/version.h>
#endif

namespace my_robot_hardware {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Sts3215HardwareInterface : public hardware_interface::SystemInterface {
 public:
#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
#else
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
#endif

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<feetech_driver::CommunicationProtocol> communication_protocol_;
  std::vector<double> hw_positions_;
  std::vector<double> state_hw_positions_;
  std::vector<double> state_hw_velocities_;
  std::vector<uint8_t> joint_ids_;
  std::vector<int> joint_offsets_;
  std::vector<int> joint_directions_;
    bool read_only_{false};
  uint64_t consecutive_read_failures_{0};
};

}  // namespace my_robot_hardware
