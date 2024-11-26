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

#ifndef TR1200_CONTROLLERS__RC_STATE_SENSOR_HPP_
#define TR1200_CONTROLLERS__RC_STATE_SENSOR_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "semantic_components/semantic_component_interface.hpp"
#include "tr1200_msgs/msg/rc_state_stamped.hpp"
#include "tr1200_msgs/msg/rc_switch_state.hpp"

namespace tr1200_rc_state_broadcaster
{

using tr1200_msgs::msg::RcStateStamped;
using tr1200_msgs::msg::RcSwitchState;

class RcStateSensor : public semantic_components::SemanticComponentInterface<RcStateStamped>
{
public:
  explicit RcStateSensor(const std::string & name)
  : SemanticComponentInterface(name, 1)
  {
    interface_names_.emplace_back(name_ + "/" + "switch_swa");
    interface_names_.emplace_back(name_ + "/" + "switch_swb");
    interface_names_.emplace_back(name_ + "/" + "switch_swc");
    interface_names_.emplace_back(name_ + "/" + "switch_swd");
    interface_names_.emplace_back(name_ + "/" + "stick_right_y");
    interface_names_.emplace_back(name_ + "/" + "stick_right_x");
    interface_names_.emplace_back(name_ + "/" + "stick_left_y");
    interface_names_.emplace_back(name_ + "/" + "stick_left_x");
    interface_names_.emplace_back(name_ + "/" + "buttons");
  }

  virtual ~RcStateSensor() = default;

  /**
   * @brief Update and return the stick states
   * @return Array of stick states in the order of stick_right_y, stick_right_x, stick_left_y,
   * stick_left_x
   */
  std::array<double, 4> get_stick_states()
  {
    for (size_t i = 0; i < 4; i++) {
      stick_states_[i] = state_interfaces_[i].get().get_value();
    }
    return stick_states_;
  }

  /**
   * @brief Update and return the switch states
   * @return Array of switch states in the order of switch_swa, switch_swb, switch_swc, switch_swd
   */
  std::array<int8_t, 4> get_switch_states()
  {
    for (size_t i = 4; i < 8; i++) {
      int switch_state_i = state_interfaces_[i].get().get_value();
      switch (switch_state_i) {
        case (0):
          switch_states_[i - 4] = RcSwitchState::RC_SWITCH_STATE_UP;
          break;
        case (1):
          switch_states_[i - 4] = RcSwitchState::RC_SWITCH_STATE_MIDDLE;
          break;
        case (2):
          switch_states_[i - 4] = RcSwitchState::RC_SWITCH_STATE_DOWN;
          break;
        default:
          switch_states_[i - 4] = RcSwitchState::RC_SWITCH_STATE_UNSET;
          break;
      }
    }
    return switch_states_;
  }

  /**
   * @brief Update and return the button states
   * @return Button states
   */
  int get_button_states()
  {
    buttons_ = state_interfaces_[8].get().get_value();
    return buttons_;
  }

  /**
   * @brief Fill RcStateStamped message with the current RC state
   * @param msg RcStateStamped message to be filled
   * @return true if the operation is successful, false otherwise
   */
  bool get_values_as_message(RcStateStamped & msg)
  {
    // Update state from state interfaces
    get_stick_states();
    get_switch_states();
    get_button_states();

    // Fill message
    msg.state.stick_right_y = stick_states_[0];
    msg.state.stick_right_x = stick_states_[1];
    msg.state.stick_left_y = stick_states_[2];
    msg.state.stick_left_x = stick_states_[3];
    msg.state.switch_swa.state = switch_states_[0];
    msg.state.switch_swb.state = switch_states_[1];
    msg.state.switch_swc.state = switch_states_[2];
    msg.state.switch_swd.state = switch_states_[3];
    msg.state.buttons = buttons_;

    return true;
  }

protected:
  std::array<double, 4> stick_states_;
  std::array<int8_t, 4> switch_states_;
  int buttons_;
};

}  // namespace tr1200_rc_state_broadcaster

#endif  // TR1200_CONTROLLERS__RC_STATE_SENSOR_HPP_
