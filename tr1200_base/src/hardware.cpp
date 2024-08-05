// Copyright 2024 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "tr1200_base/hardware.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace tr1200_base
{

CallbackReturn TR1200Interface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>(info_.name);

  try {
    port_name_ = info_.hardware_parameters.at("port_name");
  } catch (const std::out_of_range & /* e */) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Could not find port_name in hardware parameters, setting to default '%s'.",
      CAN_PORT_DEFAULT.c_str());
    port_name_ = CAN_PORT_DEFAULT;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Will connect to port '%s' on activation.",
    port_name_.c_str());

  try {
    publish_battery_state_nans_ =
      info_.hardware_parameters.at("publish_battery_state_nans") == "true";
  } catch (const std::out_of_range & /* e */) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Could not find publish_battery_state_nans in hardware parameters, setting to default of "
      "'true'.");
    publish_battery_state_nans_ = true;
  }

  // get joint names from parameters
  try {
    joint_name_left_wheel_ = info_.hardware_parameters.at("joint_name_left_wheel");
    joint_name_right_wheel_ = info_.hardware_parameters.at("joint_name_right_wheel");
  } catch (const std::out_of_range & /* e */) {
    RCLCPP_FATAL(
      node_->get_logger(),
      "Could not find joint names in hardware parameters.");
    return CallbackReturn::ERROR;
  }

  positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const auto & joint : info_.joints) {
    // only one command interface is expected for each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "Joint '%s' has %zu command interfaces found. 1 expected",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    // two state interfaces are expected for each joint
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "Joint '%s' has %zu state interfaces found. 2 expected",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces.at(0).name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "Joint '%s' has '%s' command interface found. '%s' expected",
        joint.name.c_str(), joint.command_interfaces.at(0).name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.at(0).name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "Joint '%s' has '%s' as first state interface. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(0).name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.at(1).name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        node_->get_logger(),
        "Joint '%s' has '%s' as second state interface. '%s' expected",
        joint.name.c_str(),
        joint.state_interfaces.at(1).name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TR1200Interface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints.at(i).name,
        hardware_interface::HW_IF_VELOCITY,
        &velocities_.at(i)));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints.at(i).name,
        hardware_interface::HW_IF_POSITION,
        &velocities_.at(i)));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TR1200Interface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints.at(i).name,
        hardware_interface::HW_IF_VELOCITY,
        &commands_.at(i)));
  }

  return command_interfaces;
}

CallbackReturn TR1200Interface::on_configure(const rclcpp_lifecycle::State & /*previous_states*/)
{
  try {
    driver_ = std::make_unique<tr1200_driver::TR1200Driver>();
  } catch (std::exception & e) {
    RCLCPP_FATAL(node_->get_logger(), "Failed to initialize TR1200 driver: '%s'", e.what());
    return CallbackReturn::FAILURE;
  }

  if (driver_ == NULL) {
    RCLCPP_FATAL(node_->get_logger(), "Failed to initialize TR1200 driver.");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Driver initialized.");
  driver_->initialize_state();
  if (!driver_->connect(port_name_)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Could not open and bind to CAN device '%s'.",
      port_name_.c_str());
    return CallbackReturn::ERROR;
  }

  pub_battery_state_ = node_->create_publisher<BatteryState>(
    "battery_state",
    5);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn TR1200Interface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Activating...");
  // set default values for state and command interfaces
  for (auto & position : positions_) {
    if (std::isnan(position)) {
      position = 0.0;
    }
  }
  for (auto & velocity : velocities_) {
    if (std::isnan(velocity)) {
      velocity = 0.0;
    }
  }
  for (auto & command : commands_) {
    if (std::isnan(command)) {
      command = 0.0;
    }
  }

  // Start the driver_'s read-from-CAN-thread
  RCLCPP_INFO(node_->get_logger(), "Starting driver CAN thread...");
  driver_->start();

  return CallbackReturn::SUCCESS;
}

CallbackReturn TR1200Interface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(node_->get_logger(), "Stopping driver CAN thread...");
  // Stop the driver_'s read-from-CAN-thread
  driver_->stop();
  return CallbackReturn::SUCCESS;
}

CallbackReturn TR1200Interface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_.reset();
  return CallbackReturn::SUCCESS;
}

return_type TR1200Interface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  driver_->get_motor_speeds(
    velocities_.at(0),
    velocities_.at(1));

  driver_->get_motor_positions(
    positions_.at(0),
    positions_.at(1));

  // Read and publish battery state
  auto battery_state = BatteryState();
  battery_state.header.stamp = node_->now();
  battery_state.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
  battery_state.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_state.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;

  battery_state.voltage = driver_->get_battery_voltage();
  battery_state.percentage = driver_->get_battery_soc() / 100.0f;
  battery_state.present = true;
  // TODO(lukeschmitt-tr): Reenable this once current, temp has been verified
  // battery_state.current = driver_->get_battery_current();
  // battery_state.temperature = driver_->get_battery_temperature();

  if (publish_battery_state_nans_) {
    battery_state.charge = std::numeric_limits<float>::quiet_NaN();
    battery_state.capacity = std::numeric_limits<float>::quiet_NaN();
    battery_state.design_capacity = std::numeric_limits<float>::quiet_NaN();
    battery_state.temperature = std::numeric_limits<float>::quiet_NaN();
    battery_state.current = std::numeric_limits<float>::quiet_NaN();
  } else {
    battery_state.charge = -1.0;
    battery_state.capacity = -1.0;
    battery_state.design_capacity = -1.0;
    battery_state.temperature = -1.0;
    battery_state.current = -1.0;
  }

  pub_battery_state_->publish(battery_state);

  return return_type::OK;
}

return_type TR1200Interface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  driver_->set_motor_speeds(commands_.at(0), commands_.at(1));

  return return_type::OK;
}

}  // namespace tr1200_base

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  tr1200_base::TR1200Interface,
  hardware_interface::SystemInterface)
