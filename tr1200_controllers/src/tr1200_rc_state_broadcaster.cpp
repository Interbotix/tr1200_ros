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

#include <string>

#include "tr1200_controllers/tr1200_rc_state_broadcaster.hpp"

namespace tr1200_rc_state_broadcaster
{
CallbackReturn TR1200RcStateBroadcaster::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during init stage with message: %s",
      e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration TR1200RcStateBroadcaster::command_interface_configuration() const
{
  InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

InterfaceConfiguration TR1200RcStateBroadcaster::state_interface_configuration() const
{
  InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = rc_state_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

CallbackReturn
TR1200RcStateBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  if (params_.sensor_name.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'sensor_name' parameter is empty.");
    return CallbackReturn::ERROR;
  }
  if (params_.frame_id.empty()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "'frame_id' parameter is empty. Using 'base_link' as default.");
    // TODO(lukeschmitt-tr): Set frame_id to base_link here
  }
  if (params_.rc_type.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'rc_type' parameter is empty.");
    return CallbackReturn::ERROR;
  }

  rc_state_sensor_ = std::make_unique<RcStateSensor>(params_.sensor_name);

  try {
    pub_rc_state_ = get_node()->create_publisher<RcStateStamped>(
      "~/rc_state",
      rclcpp::SystemDefaultsQoS());
    rt_pub_rc_state_ = std::make_unique<realtime_tools::RealtimePublisher<RcStateStamped>>(
      pub_rc_state_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage with message : %s",
      e.what());
    return CallbackReturn::ERROR;
  }

  rt_pub_rc_state_->lock();
  if (params_.frame_id.empty()) {
    rt_pub_rc_state_->msg_.header.frame_id = "base_link";
  } else {
    rt_pub_rc_state_->msg_.header.frame_id = params_.frame_id;
  }
  rt_pub_rc_state_->msg_.state.rc_type = params_.rc_type;
  rt_pub_rc_state_->unlock();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "TR1200RcStateBroadcaster configured successfully.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
TR1200RcStateBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  rc_state_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn
TR1200RcStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  rc_state_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

return_type
TR1200RcStateBroadcaster::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (rt_pub_rc_state_ && rt_pub_rc_state_->trylock()) {
    rc_state_sensor_->get_values_as_message(rt_pub_rc_state_->msg_);
    rt_pub_rc_state_->msg_.header.stamp = time;
    rt_pub_rc_state_->unlockAndPublish();
  }

  return return_type::OK;
}

}  // namespace tr1200_rc_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tr1200_rc_state_broadcaster::TR1200RcStateBroadcaster,
  controller_interface::ControllerInterface)
