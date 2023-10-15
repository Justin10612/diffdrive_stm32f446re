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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "diffdrive_stm32f446re/diffdrive_stm32f446re.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <rclcpp_components/register_node_macro.hpp>


namespace diffdrive_stm32f446re_hardware
{

VelocityPublisher::VelocityPublisher() : Node("stm32_hwif")
{
  publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("wheel_cmd_vel", 10);
}

void VelocityPublisher::publishData(float l_input, float r_input)
{
  auto message = geometry_msgs::msg::Vector3();
  message.x = l_input;
  message.y = r_input;
  publisher_->publish(message);
}


hardware_interface::return_type DiffBotSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  //fire up the publisher node
  hw_rps_pub_ = std::make_shared<VelocityPublisher>(); 

  cfg_.left_wheel_name  = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate  = stof(info_.hardware_parameters["loop_rate"]);
  cfg_.enc_counts_per_rev  = stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // wheel state interface:pos
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));

  // state interface:wheel_vel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // command interface:wheel cmd_vel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::return_type DiffBotSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Starting ...please wait...");
  // Follow the toturial: //comms_.connect();
  status_ = hardware_interface::status::STARTED;
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "System Successfully started!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Stopping ...please wait...");
  // Follow the toturial: //comms_.disconnect();
  status_ = hardware_interface::status::STOPPED;
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "System successfully stopped!");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::read()
{
  // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Reading...");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_stm32f446re_hardware::DiffBotSystemHardware::write()
{
  // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");
  float l_rps = wheel_l_.cmd;
  float r_rps = wheel_r_.cmd;
  hw_rps_pub_ -> publishData(l_rps, r_rps);
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_stm32f446re_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diffdrive_stm32f446re_hardware::DiffBotSystemHardware, hardware_interface::SystemInterface)