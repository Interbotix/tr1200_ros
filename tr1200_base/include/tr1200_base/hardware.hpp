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

#ifndef TR1200_BASE__HARDWARE_HPP_
#define TR1200_BASE__HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tr1200_base/driver.hpp"

namespace tr1200_base
{

const inline static std::string CAN_PORT_DEFAULT = "can0";

using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using sensor_msgs::msg::BatteryState;

class TR1200Interface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TR1200Interface)

  /// @brief Initialize the TR1200Interface
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /// @brief Exports all state interfaces for the TR1200Interface.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// @brief Exports all command interfaces for the TR1200Interface.
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  /// @brief Read state information
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// @brief Write commands
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// @brief Returns the name of this hardware interface
  /// @return The name of this hardware interface
  std::string get_name() const final
  {
    return info_.name;
  }

protected:
  // ROS Node for this hardware interface's pubs, subs, and services
  std::shared_ptr<rclcpp::Node> node_;

  // Pointer to the TR1200 Driver created by this hardware interface
  std::unique_ptr<tr1200_driver::TR1200Driver> driver_{nullptr};

  // Name of the left wheel joint
  std::string joint_name_left_wheel_;

  // Name of the right wheel joint
  std::string joint_name_right_wheel_;

  // Name of the CAN bus port the TR1200 driver should connect to
  std::string port_name_{CAN_PORT_DEFAULT};

  // Logging level of the TR1200 driver
  std::string driver_logging_level_{"INFO"};

  // Wheel positions [rad]
  std::vector<double> positions_;

  // Wheel velocities [rad/s]
  std::vector<double> velocities_;

  // Wheel velocity commands [rad]
  std::vector<double> commands_;

  // Battery state publisher
  rclcpp::Publisher<BatteryState>::SharedPtr pub_battery_state_;

  // True to publish unmeasured battery state values as NaNs, false to publish -1s
  bool publish_battery_state_nans_{true};
};

}  // namespace tr1200_base

#endif  // TR1200_BASE__HARDWARE_HPP_
