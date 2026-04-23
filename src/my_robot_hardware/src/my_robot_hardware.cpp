#include "my_robot_hardware/my_robot_hardware.hpp"

#include <feetech_driver/SMS_STS.h>
#include <feetech_driver/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace my_robot_hardware {

#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
CallbackReturn Sts3215HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
#else
CallbackReturn Sts3215HardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
#endif

  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"),
                 "Hardware parameter [usb_port] is missing. Please add <param name=\"usb_port\">/dev/ttyUSB0</param>.");
    return CallbackReturn::ERROR;
  }

  if (const auto read_only_it = info_.hardware_parameters.find("read_only"); read_only_it != info_.hardware_parameters.end()) {
    const auto& value = read_only_it->second;
    read_only_ = (value == "true" || value == "1" || value == "TRUE");
  }

  auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_it->second);
  if (const auto result = serial_port->configure(); !result) {
    // Work around third-party SerialPort destructor throwing when port is not open.
    // In this failure path we intentionally release to avoid process abort.
    (void)serial_port.release();
    RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Serial port configuration failed: %s", result.error().c_str());
    return CallbackReturn::ERROR;
  }

  communication_protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));

  joint_ids_.resize(info_.joints.size());
  joint_offsets_.resize(info_.joints.size());
  joint_directions_.resize(info_.joints.size(), 1);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint_params = info_.joints[i].parameters;
    const auto id_it = joint_params.find("id");
    if (id_it == joint_params.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Joint '%s' misses parameter 'id'.", info_.joints[i].name.c_str());
      return CallbackReturn::ERROR;
    }

    try {
      joint_ids_[i] = static_cast<uint8_t>(std::stoi(id_it->second));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Invalid id for joint '%s': %s",
                   info_.joints[i].name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }

    joint_offsets_[i] = 0;
    if (const auto offset_it = joint_params.find("offset"); offset_it != joint_params.end()) {
      try {
        joint_offsets_[i] = std::stoi(offset_it->second);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Invalid offset for joint '%s': %s",
                     info_.joints[i].name.c_str(), e.what());
        return CallbackReturn::ERROR;
      }
    }

    if (const auto direction_it = joint_params.find("direction"); direction_it != joint_params.end()) {
      try {
        const auto direction_value = std::stoi(direction_it->second);
        if (direction_value != 1 && direction_value != -1) {
          RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"),
                       "Invalid direction for joint '%s': %d (expected 1 or -1)",
                       info_.joints[i].name.c_str(), direction_value);
          return CallbackReturn::ERROR;
        }
        joint_directions_[i] = direction_value;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Invalid direction for joint '%s': %s",
                     info_.joints[i].name.c_str(), e.what());
        return CallbackReturn::ERROR;
      }
    }

    for (const auto& [parameter_name, address] : {std::pair{"p_cofficient", SMS_STS_P_COEF},
                                                  std::pair{"i_cofficient", SMS_STS_I_COEF},
                                                  std::pair{"d_cofficient", SMS_STS_D_COEF}}) {
      if (const auto param_it = joint_params.find(parameter_name); param_it != joint_params.end()) {
        uint8_t pid_value = 0;
        try {
          const auto parsed = std::stod(param_it->second);
          const auto rounded = static_cast<int>(std::lround(parsed));
          if (rounded < 0 || rounded > 255) {
            RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"),
                         "%s for joint '%s' is out of range [0,255]: %d", parameter_name,
                         info_.joints[i].name.c_str(), rounded);
            return CallbackReturn::ERROR;
          }
          pid_value = static_cast<uint8_t>(rounded);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Invalid %s for joint '%s': %s",
                       parameter_name, info_.joints[i].name.c_str(), e.what());
          return CallbackReturn::ERROR;
        }

        const auto result = communication_protocol_->write(
            joint_ids_[i], address, std::experimental::make_array(pid_value));
        if (!result) {
          RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Failed to set %s for joint '%s': %s",
                       parameter_name, info_.joints[i].name.c_str(), result.error().c_str());
          return CallbackReturn::ERROR;
        }
      }
    }

    if (info_.joints[i].command_interfaces.empty()) {
      if (const auto result = communication_protocol_->set_torque(joint_ids_[i], false); !result) {
        RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Failed to disable torque for joint '%s': %s",
                     info_.joints[i].name.c_str(), result.error().c_str());
        return CallbackReturn::ERROR;
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Sts3215HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_hw_positions_.assign(info_.joints.size(), 0.0);
  state_hw_velocities_.assign(info_.joints.size(), 0.0);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Sts3215HardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  hw_positions_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    if (!info_.joints[i].command_interfaces.empty()) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    }
  }

  return command_interfaces;
}

hardware_interface::return_type Sts3215HardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::vector<std::array<uint8_t, 4>> data;
  data.reserve(joint_ids_.size());

  if (auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data); !result) {
    consecutive_read_failures_++;

    // Fallback: read each joint independently to avoid freezing /joint_states on occasional sync read errors.
    bool any_joint_updated = false;
    for (std::size_t i = 0; i < joint_ids_.size(); ++i) {
      const auto position = communication_protocol_->read_position(joint_ids_[i]);
      const auto speed = communication_protocol_->read_speed(joint_ids_[i]);
      if (position && speed) {
        state_hw_positions_[i] = static_cast<double>(joint_directions_[i]) *
                                 feetech_driver::to_radians(position.value() - joint_offsets_[i]);
        state_hw_velocities_[i] = static_cast<double>(joint_directions_[i]) * feetech_driver::to_radians(speed.value());
        any_joint_updated = true;
      }
    }

    if (consecutive_read_failures_ % 100 == 1) {
      RCLCPP_WARN(rclcpp::get_logger("Sts3215HardwareInterface"),
                  "sync_read failed (%llu times). Fallback per-joint read %s. Last error: %s",
                  static_cast<unsigned long long>(consecutive_read_failures_),
                  any_joint_updated ? "succeeded" : "failed", result.error().c_str());
    }
    return hardware_interface::return_type::OK;
  }

  consecutive_read_failures_ = 0;

  for (std::size_t i = 0; i < data.size(); ++i) {
    const auto raw_position = feetech_driver::from_sts(feetech_driver::WordBytes{.low = data[i][0], .high = data[i][1]});
    const auto raw_speed = feetech_driver::from_sts(feetech_driver::WordBytes{.low = data[i][2], .high = data[i][3]});
    state_hw_positions_[i] = static_cast<double>(joint_directions_[i]) *
                             feetech_driver::to_radians(raw_position - joint_offsets_[i]);
    state_hw_velocities_[i] = static_cast<double>(joint_directions_[i]) * feetech_driver::to_radians(raw_speed);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Sts3215HardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (read_only_) {
    return hardware_interface::return_type::OK;
  }

  std::vector<uint8_t> commanded_joint_ids;
  std::vector<int> commanded_positions;
  std::vector<int> commanded_speeds;
  std::vector<int> commanded_accelerations;

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    if (!info_.joints[i].command_interfaces.empty()) {
      commanded_joint_ids.push_back(joint_ids_[i]);
      commanded_positions.push_back(feetech_driver::from_radians(static_cast<double>(joint_directions_[i]) * hw_positions_[i]) +
                                    joint_offsets_[i]);
      commanded_speeds.push_back(2400);
      commanded_accelerations.push_back(50);
    }
  }

  if (!commanded_joint_ids.empty()) {
    if (auto result = communication_protocol_->sync_write_position(commanded_joint_ids, commanded_positions,
                                                                  commanded_speeds, commanded_accelerations);
        !result) {
      RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Write failed: %s", result.error().c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn Sts3215HardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0)) != hardware_interface::return_type::OK) {
    return CallbackReturn::ERROR;
  }
  hw_positions_ = state_hw_positions_;

  if (read_only_) {
    std::vector<std::array<uint8_t, 1>> torque_off(joint_ids_.size());
    for (auto& parameter : torque_off) {
      parameter[0] = 0;
    }

    if (auto result = communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, torque_off); !result) {
      RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Failed to disable torque in read-only mode: %s",
                   result.error().c_str());
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Sts3215HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  std::vector<std::array<uint8_t, 1>> torque_off(joint_ids_.size());
  for (auto& parameter : torque_off) {
    parameter[0] = 0;
  }

  if (auto result = communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, torque_off); !result) {
    RCLCPP_ERROR(rclcpp::get_logger("Sts3215HardwareInterface"), "Failed to disable torque: %s", result.error().c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::Sts3215HardwareInterface, hardware_interface::SystemInterface)
