// Copyright 2020 ros2_control Development Team
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

#include "ros2_control_hand/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_hand
{
hardware_interface::CallbackReturn CustomHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_position.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HardwareInterface"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CustomHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("CustomHardware"), "Configuring ...please wait...");

  // reset values always when configuring hardware
  for (uint i = 0; i < hw_states_position.size(); i++)
  {
    hw_states_position[i] = 0;
    hw_states_velocity[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CustomHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("HardwareInterface"), "Activating ...please wait...");

  std::string port = "/dev/ttyACM0";
  SerialPort = open(port.c_str(), O_RDWR);
  if(SerialPort < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Error %i from open: %s", errno, strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  if(tcgetattr(SerialPort, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Error %i from tcgetattr: %s", errno, strerror(errno));
    close(SerialPort);
    return hardware_interface::CallbackReturn::ERROR;
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_lflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;

  speed_t speed = B115200;
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tcflush(SerialPort, TCIOFLUSH);

  if(tcsetattr(SerialPort, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HardwareInterface"), "Error %i from tcgetattr: %s", errno, strerror(errno));
    close(SerialPort);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Wait to use serial port
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "SERIAL PORT OPENED: %d! WAITING...", SerialPort);
  auto t_start = std::chrono::high_resolution_clock::now();
  while(true)
  {
    auto t_end = std::chrono::high_resolution_clock::now();
    auto elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    if (elapsed_time_ms > 3000)
    {
      break;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CustomHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("HardwareInterface"), "Deactivating ...please wait...");

  if(SerialPort == -1)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  tcflush(SerialPort, TCIOFLUSH);
  close(SerialPort);

  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>CustomHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_velocity[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>CustomHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

int CustomHardware::WriteToSerial(unsigned char* buf, int nBytes)
{
  return ::write(SerialPort, buf, nBytes);
}

int CustomHardware::ReadSerial(unsigned char* buf, int nBytes)
{
  auto t_start = std::chrono::high_resolution_clock::now();
  int n = 0;
  while(n < nBytes)
  {
    int ret = ::read(SerialPort, &(buf[n]), 1);
    if (ret < 0)
    {
      return ret;
    }
    n += ret;
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    if (elapsed_time_ms > 10000)
    {
      break;
    }
  }
  return n;
}


hardware_interface::return_type CustomHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration &)
{
  RCLCPP_INFO(rclcpp::get_logger("CustomHardware"), "Reading...");

  unsigned char r[1] = {'r'};
  WriteToSerial(r, 1);
  float ret[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  uint8_t* v = (uint8_t*)ret;
  ReadSerial(v, sizeof(ret));
  RCLCPP_INFO(rclcpp::get_logger("HardwareInterface"), "Received : %f, %f, %f, %f, %f, %f", 
    ret[0], ret[1], ret[2], ret[3], ret[4], ret[5]);
  for (uint i = 0; i < hw_states_position.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_position[i] = ret[i];
    //print joint name

    // Print joint name
    const std::string& joint_name = info_.joints[i].name;
    RCLCPP_INFO(
        rclcpp::get_logger("CustomHardware"), "Got state %.5f for joint %s!",
        hw_states_position[i], joint_name.c_str());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CustomHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_hand

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_hand::CustomHardware, hardware_interface::SystemInterface)
