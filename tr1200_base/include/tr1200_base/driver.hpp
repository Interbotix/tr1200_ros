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

#ifndef TR1200_BASE__DRIVER_HPP_
#define TR1200_BASE__DRIVER_HPP_

#include <linux/can.h>
#include <linux/input.h>
#include <net/if.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "tr1200_driver/can.hpp"
#include "tr1200_driver/driver_version.hpp"
#include "tr1200_driver/logging.hpp"

namespace tr1200_driver
{

//// Feedback/State message IDs
// Motor state feedback message IDs
const int CAN_ID_FB_MOTOR_STATE = 0x200;

// Battery management system feedback message ID
const int CAN_ID_FB_BMS_STATE = 0x281;

// FlySky remote control transmitter state feedback message ID
const int CAN_ID_FB_FS_RC_STATE = 0x341;

// Kar-Tech remote control transmitter state feedback message ID
const int CAN_ID_FB_KT_RC_STATE = 0x351;

// Fault status feedback message ID
const int CAN_ID_FB_FAULT_STATUS = 0x151;

//// Command/Control message IDs
// Motor speed command message ID
const int CAN_ID_CTRL_MOTOR_VEL = 0x300;

// Firmware version request message ID
const int CAN_ID_CTRL_FW_VERSION_REQ = 0x321;

// Firmware version response message ID
const int CAN_ID_CTRL_FW_VERSION_RES = 0x331;

// Clear fault status command message ID
const int CAN_ID_CTRL_CLEAR_FAULT_STATUS = 0x251;

// Default name of CAN port
const inline static std::string CAN_PORT = "can0";

const inline static float TWO_PI = 2.0f * 3.14159265359f;
const inline static float RPM_TO_RAD_PER_SEC = TWO_PI / 60.0f;

class TR1200Driver
{
public:
  /**
   * @brief Construct the TR1200Driver object
   */
  TR1200Driver();
  ~TR1200Driver();

  bool connect(const std::string port_name);
  void initialize_state();
  void start();
  void stop();

  /**
   * @brief Set motor speeds
   * @param speeds The desired motor speeds in rad/s
   */
  void set_motor_speeds(std::array<double, 2> speeds);

  /**
   * @brief Set motor speeds
   * @param speed_l The desired left motor speeds in rad/s
   * @param speed_r The desired right motor speeds in rad/s
   */
  void set_motor_speeds(const double speed_l, const double speed_r);

  /**
   * @brief Get motor speeds
   * @param speed_l[out] The current left motor speed in rad/s
   * @param speed_r[out] The current right motor speed in rad/s
   */
  void get_motor_speeds(double & speed_l, double & speed_r);

  /**
   * @brief Get motor positions
   * @param position_l[out] The current left motor position in rad
   * @param position_r[out] The current right motor position in rad
   */
  void get_motor_positions(double & position_l, double & position_r);

  /**
   * @brief Get the battery's state of charge as reported by the BMS
   * @return The battery's state of charge in %
   */
  float get_battery_soc();

  /**
   * @brief Get the battery's voltage as reported by the BMS
   * @return The battery's voltage in V
   */
  float get_battery_voltage();

  /**
   * @brief Get the battery's current as reported by the BMS
   * @return The battery's current in A
   */
  float get_battery_current();

  /**
   * @brief Get the battery's temperature as reported by the BMS
   * @return The battery's temperature in C
   */
  float get_battery_temperature();

  /**
   * @brief Get the base's firmware version
   * @param[out] major The base's firmware major version
   * @param[out] minor The base's firmware minor version
   * @param[out] patch The base's firmware patch version
   */
  void get_firmware_version(uint8_t & major, uint8_t & minor, uint8_t & patch);

  /**
   * @brief Get the state of the RC sticks
   * @param left_x[out] The state of stick left_x
   * @param left_y[out] The state of stick left_y
   * @param right_x[out] The state of stick right_x
   * @param right_y[out] The state of stick right_y
   */
  void get_rc_stick_states(
    int8_t & left_x,
    int8_t & left_y,
    int8_t & right_x,
    int8_t & right_y);

  /**
   * @brief Get the state of the RC knobs
   * @param vra[out] The state of knob vra
   * @param vrb[out] The state of knob vrb
   */
  void get_rc_knob_states(int8_t & vra, int8_t & vrb);

  /**
   * @brief Get the state of the RC switches
   * @param swa[out] The state of switch swa
   * @param swb[out] The state of switch swb
   * @param swc[out] The state of switch swc
   * @param swd[out] The state of switch swd
   */
  void get_rc_switch_states(
    int8_t & swa,
    int8_t & swb,
    int8_t & swc,
    int8_t & swd);

  /**
   * @brief Get the state of the RC buttons
   * @param buttons[out] The state of the buttons
   */
  void get_rc_button_states(int8_t & buttons);

  /**
   * @brief Get the RC type
   * @param rc_type[out] The RC type
   */
  void get_rc_type(std::string & rc_type);

protected:
  std::mutex mutex_cmd_;
  std::mutex mutex_bms_;
  std::mutex mutex_motor_fb_;
  std::mutex mutex_firmware_;
  std::mutex mutex_fault_;
  std::mutex mutex_rc_state_;

  std::thread thread_read_from_can_;

  // CANbus communication interface
  int dev_handler_;

  bool run_;

  RcState rc_state_;
  FaultStatus fault_status_;
  std::string rc_type_{"UNKNOWN"};
  std::array<ControlMotorSpeed, 2> control_motor_speed_;
  std::array<FeedbackMotorState, 2> feedback_motor_state_;
  BmsState bms_state_;
  FirmwareVersion firmware_version_;

  /**
   * @brief Main CAN parsing loop
   */
  void read_can();

  /**
   * @brief Write CAN frame to robot's CAN bus
   * @param frame CAN frame to write
   */
  void write_can(const can_frame & frame);

  /**
   * @brief Validate checksum
   * @param frame CAN frame to validate
   * @return true if checksum is validated, false otherwise
   */
  bool validate_checksum(unsigned char frame[8]);

  /**
   * @brief Calculate and encode checksum
   * @param frame CAN frame to calculate and encode checksum into
   */
  void encode_checksum(unsigned char * frame);

  /**
   * @brief Decode a CAN frame to a TR message
   * @param frame constant pointer to the CAN frame to decode
   */
  void decode_can_frame(const can_frame & recv_frame);

  /**
   * @brief Encode a TR message to a CAN frame
   * @param out_frame reference to the CAN frame to encode TR message to
   * @param tr_msg constant reference to TR message to encode
   * @return true if encoding was successful, false otherwise
   */
  void encode_fw_can_frame(can_frame & out_frame);

  /**
   * @brief Encode a TR message to a CAN frame
   * @param out_frame reference to the CAN frame to encode TR message to
   * @param tr_msg constant reference to TR message to encode
   * @return true if encoding was successful, false otherwise
   */
  void encode_cmd_can_frame(can_frame & out_frame, const ControlMotorSpeed & ctrl_speed);

  /**
   * @brief Update a motor's state using its feedback message
   * @param motor_state A motor's incoming feedback message
   */
  void update_motor_state(const FeedbackMotorState & motor_state);

  /**
   * @brief Update the BMS's state using its feedback message
   * @param bms_state The BMS's incoming feedback message
   */
  void update_bms_state(const BmsState & bms_state);

  /**
   * @brief Update the RC's state using its feedback message
   * @param rc_state The RC's incoming feedback message
   */
  void update_rc_state(const RcState & rc_state);

  /**
   * @brief Update the fault state using its feedback message
   * @param fault_state The fault incoming feedback message
   */
  void update_fault_status(const FaultStatus & fault_status);

  /**
   * @brief Update the firmware version using its response message
   * @param motor_state The fault incoming response message
   */
  void update_firmware_version(const FirmwareVersion & version);

  /**
   * @brief Print CAN frame in a human-readable format
   * @param frame CAN frame to print
   */
  void print_can_frame(const can_frame & frame);

  /**
   * @brief Print CAN frame in a human-readable format
   * @param frame CAN frame to print
   */
  void print_can_frame(const can_frame * frame);
};

}  // namespace tr1200_driver

#endif  // TR1200_BASE__DRIVER_HPP_
