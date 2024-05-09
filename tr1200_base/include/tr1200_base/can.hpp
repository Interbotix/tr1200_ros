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

#ifndef TR1200_BASE__CAN_HPP_
#define TR1200_BASE__CAN_HPP_

#include <stdint.h>

namespace tr1200_driver
{

/**
 * @brief Enum type for switch states
 */
enum RcSwitchState
{
  // switch is up
  RC_SWITCH_STATE_UP = 0,

  // switch is in the middle
  RC_SWITCH_STATE_MIDDLE = 1,

  // switch is down
  RC_SWITCH_STATE_DOWN = 2
};

/**
 * @brief Struct type for RC state
 */
typedef struct RcState RcState;
struct RcState
{
  // switch A state
  RcSwitchState switch_swa;

  // switch B state
  RcSwitchState switch_swb;

  // switch C state
  RcSwitchState switch_swc;

  // switch D state
  RcSwitchState switch_swd;

  // Right stick vertical axis
  int8_t stick_right_y;

  // Right stick horizontal axis
  int8_t stick_right_x;

  // Left stick vertical axis
  int8_t stick_left_y;

  // Left stick horizontal axis
  int8_t stick_left_x;

  // VRA knob
  int8_t knob_vra;
};

typedef struct ControlMotorSpeed ControlMotorSpeed;
struct ControlMotorSpeed
{
  // motor id
  uint8_t id;

  // motor velocity in 0.01 rad/s of drive sprocket shaft
  int16_t velocity;
};

typedef struct ControlMotorPosition ControlMotorPosition;
struct ControlMotorPosition
{
  // motor id
  uint8_t id;

  // motor velocity in 0.001 rad
  int16_t position;
};

typedef struct FeedbackMotorState FeedbackMotorState;
struct FeedbackMotorState
{
  // motor id
  uint8_t id;

  // motor velocity in 0.01 rad/s of drive sprocket shaft
  int16_t velocity;

  // counts
  int32_t encoder_count;
};

typedef struct BmsState BmsState;
struct BmsState
{
  // voltage in units of 0.1 V range of [0.0 V - 65.5 V]
  uint16_t voltage;

  // state of change in units of 0.1% range of 0% to 100%
  uint16_t soc;

  // current offset in units of 0.1 A
  uint16_t current;

  // Temperature offset from 40°C in units of °C
  int8_t temperature;
};

typedef struct FaultStatus FaultStatus;
struct FaultStatus
{
  // true if fault exists, false otherwise
  bool fault;
};

typedef struct FirmwareVersion FirmwareVersion;
struct FirmwareVersion
{
  // Firmware major version
  int8_t major;

  // Firmware minor version
  int8_t minor;

  // Firmware patch version
  int8_t patch;
};


enum MessageType
{
  MESSAGE_TYPE_UNKNOWN = 0x00,
  MESSAGE_TYPE_RC_STATE,
  MESSAGE_TYPE_CONTROL_MOTOR_SPEED,
  MESSAGE_TYPE_CONTROL_MOTOR_POSITION,
  MESSAGE_TYPE_FEEDBACK_MOTOR_STATE,
  MESSAGE_TYPE_BMS_STATE,
  MESSAGE_TYPE_FAULT_STATUS,
  MESSAGE_TYPE_FIRMWARE_VERSION,
};

}  // namespace tr1200_driver

#endif  // TR1200_BASE__CAN_HPP_
