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

#ifndef TR1200_CONTROLLERS__TR1200_RC_STATE_BROADCASTER_HPP_
#define TR1200_CONTROLLERS__TR1200_RC_STATE_BROADCASTER_HPP_

#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "tr1200_controllers/rc_state_sensor.hpp"
#include "tr1200_msgs/msg/rc_state_stamped.hpp"
#include "tr1200_rc_state_broadcaster_parameters.hpp"

namespace tr1200_rc_state_broadcaster
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using tr1200_msgs::msg::RcStateStamped;

class TR1200RcStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  InterfaceConfiguration command_interface_configuration() const override;

  InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::unique_ptr<tr1200_rc_state_broadcaster::RcStateSensor> rc_state_sensor_;

  rclcpp::Publisher<RcStateStamped>::SharedPtr pub_rc_state_;
  std::unique_ptr<realtime_tools::RealtimePublisher<RcStateStamped>> rt_pub_rc_state_;
};
}  // namespace tr1200_rc_state_broadcaster

#endif  // TR1200_CONTROLLERS__TR1200_RC_STATE_BROADCASTER_HPP_
