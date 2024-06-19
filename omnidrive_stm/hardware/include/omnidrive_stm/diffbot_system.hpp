// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OMNIDRIVE_STM__DIFFBOT_SYSTEM_HPP_
#define OMNIDRIVE_STM__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "omnidrive_stm/visibility_control.h"
#include "omnidrive_stm/stm_comms.hpp"
#include "omnidrive_stm/wheel.hpp"

namespace omnidrive_stm
{
class OmniDriveStmHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string wheel_0 = "";
  std::string wheel_1 = "";
  std::string wheel_2 = "";
  std::string wheel_3 = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  // int pid_p = 0;
  // int pid_d = 0;
  // int pid_i = 0;
  // int pid_o = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmniDriveStmHardware);

  OMNIDRIVE_STM_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  OMNIDRIVE_STM_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  OMNIDRIVE_STM_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  OMNIDRIVE_STM_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNIDRIVE_STM_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNIDRIVE_STM_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  OMNIDRIVE_STM_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  StmComms comms_;
  Config cfg_;
  Wheel wheel_0_;
  Wheel wheel_1_;
  Wheel wheel_2_;
  Wheel wheel_3_;
};
}  // namespace omnidrive_stm

#endif  // OMNIDRIVE_STM__DIFFBOT_SYSTEM_HPP_
