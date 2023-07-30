// Copyright 2022 ROBOTIS CO., LTD.
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
//
// Author: Darby Lim

#include "turtlebot3_hardware/opencr.hpp"

#include <unistd.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace robotis
{
namespace turtlebot3_hardware
{
OpenCR::OpenCR(const uint8_t & id)
{
  dxl_sdk_wrapper_ = std::make_unique<DynamixelSDKWrapper>(id);
}

OpenCR::~OpenCR()
{
  send_heartbeat(1);
  wheels_torque(opencr::OFF);
}

bool OpenCR::open_port(const std::string & usb_port)
{
  return dxl_sdk_wrapper_->open_port(usb_port);
}

bool OpenCR::set_baud_rate(const uint32_t & baud_rate)
{
  return dxl_sdk_wrapper_->set_baud_rate(baud_rate);
}

uint16_t OpenCR::ping()
{
  return dxl_sdk_wrapper_->ping();
}

bool OpenCR::is_connect_wheels()
{
  return dxl_sdk_wrapper_->read_byte(opencr_control_table.connect_wheels.address);
}

void OpenCR::play_sound(uint8_t sound) const
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.sound.address, sound);
}

void OpenCR::imu_recalibration()
{
  dxl_sdk_wrapper_->write_byte(
    opencr_control_table.imu_re_calibration.address,
    opencr_control_table.imu_re_calibration.length);
}

opencr::IMU OpenCR::get_imu()
{
  opencr::IMU imu;

  imu.angular_velocity.x = get_data<float>(
    opencr_control_table.imu_angular_velocity_x.address,
    opencr_control_table.imu_angular_velocity_x.length);

  imu.angular_velocity.y = get_data<float>(
    opencr_control_table.imu_angular_velocity_y.address,
    opencr_control_table.imu_angular_velocity_y.length);

  imu.angular_velocity.z = get_data<float>(
    opencr_control_table.imu_angular_velocity_z.address,
    opencr_control_table.imu_angular_velocity_z.length);

  imu.linear_acceleration.x = get_data<float>(
    opencr_control_table.imu_linear_acceleration_x.address,
    opencr_control_table.imu_linear_acceleration_x.length);

  imu.linear_acceleration.y = get_data<float>(
    opencr_control_table.imu_linear_acceleration_y.address,
    opencr_control_table.imu_linear_acceleration_y.length);

  imu.linear_acceleration.z = get_data<float>(
    opencr_control_table.imu_linear_acceleration_z.address,
    opencr_control_table.imu_linear_acceleration_z.length);

  imu.orientation.x = get_data<float>(
    opencr_control_table.imu_orientation_x.address,
    opencr_control_table.imu_orientation_x.length);

  imu.orientation.y = get_data<float>(
    opencr_control_table.imu_orientation_y.address,
    opencr_control_table.imu_orientation_y.length);

  imu.orientation.z = get_data<float>(
    opencr_control_table.imu_orientation_z.address,
    opencr_control_table.imu_orientation_z.length);

  imu.orientation.w = get_data<float>(
    opencr_control_table.imu_orientation_w.address,
    opencr_control_table.imu_orientation_w.length);

  return imu;
}

opencr::Battery OpenCR::get_battery()
{
  opencr::Battery battery;

  battery.design_capacity = 1.8;

  battery.voltage = 0.01 * get_data<int32_t>(
    opencr_control_table.battery_voltage.address,
    opencr_control_table.battery_voltage.length);

  battery.percentage = 0.01 * get_data<int32_t>(
    opencr_control_table.battery_percentage.address,
    opencr_control_table.battery_percentage.length);

  battery.voltage <= 7.0 ? battery.present = false : battery.present = true;

  return battery;
}

void OpenCR::wheels_torque(uint8_t onoff) const
{
  dxl_sdk_wrapper_->write_byte(opencr_control_table.torque_wheels.address, onoff);
}

bool OpenCR::read_all()
{
  bool comm_result = dxl_sdk_wrapper_->read(0, CONTROL_TABLE_SIZE, &data_buffer_[0]);

  if (comm_result) {
    std::lock_guard<std::mutex> lock(buffer_m_);
    std::copy(data_buffer_, data_buffer_ + CONTROL_TABLE_SIZE, data_);
  }

  return comm_result;
}

std::array<double, 2> OpenCR::get_wheel_positions()
{
  static std::array<int32_t, 2> last_diff_ticks = {0, 0};
  static std::array<int32_t, 2> last_ticks = {0, 0};
  std::array<double, 2> positions = {0.0, 0.0};

  std::array<int32_t, 2> ticks = {
    get_data<int32_t>(
      opencr_control_table.present_position_left.address,
      opencr_control_table.present_position_left.length),
    get_data<int32_t>(
      opencr_control_table.present_position_right.address,
      opencr_control_table.present_position_right.length)
  };

  positions[opencr::wheels::LEFT] =
    opencr::wheels::TICK_TO_RAD * last_diff_ticks[opencr::wheels::LEFT];
  positions[opencr::wheels::RIGHT] =
    opencr::wheels::TICK_TO_RAD * last_diff_ticks[opencr::wheels::RIGHT];

  last_diff_ticks[opencr::wheels::LEFT] +=
    (ticks[opencr::wheels::LEFT] - last_ticks[opencr::wheels::LEFT]);
  last_diff_ticks[opencr::wheels::RIGHT] +=
    (ticks[opencr::wheels::RIGHT] - last_ticks[opencr::wheels::RIGHT]);

  last_ticks = ticks;

  return positions;
}

std::array<double, 2> OpenCR::get_wheel_velocities()
{
  std::array<double, 2> velocities = {0.0, 0.0};

  std::array<int32_t, 2> rpms = {
    get_data<int32_t>(
      opencr_control_table.present_velocity_left.address,
      opencr_control_table.present_velocity_left.length),
    get_data<int32_t>(
      opencr_control_table.present_velocity_right.address,
      opencr_control_table.present_velocity_right.length)
  };

  velocities[opencr::wheels::LEFT] = opencr::wheels::RPM_TO_MS * rpms[opencr::wheels::LEFT];
  velocities[opencr::wheels::RIGHT] = opencr::wheels::RPM_TO_MS * rpms[opencr::wheels::RIGHT];

  return velocities;
}

bool OpenCR::set_wheel_velocities(const std::vector<double> & velocities)
{
  union Data {
    int32_t dword[6];
    uint8_t byte[6 * 4];
  } data;

  double left_wheel_velocity = velocities[0] * opencr::wheels::RADIUS;
  double right_wheel_velocity = velocities[1] * opencr::wheels::RADIUS;

  double linear_velocity_x = (right_wheel_velocity + left_wheel_velocity) / 2.0;
  double angular_velocity_z =
    (right_wheel_velocity - left_wheel_velocity) / opencr::wheels::SEPERATION;

  data.dword[0] = static_cast<int32_t>(linear_velocity_x * 100);
  data.dword[1] = 0;
  data.dword[2] = 0;
  data.dword[3] = 0;
  data.dword[4] = 0;
  data.dword[5] = static_cast<int32_t>(angular_velocity_z * 100);

  uint8_t * p_data = &data.byte[0];
  bool comm_result = dxl_sdk_wrapper_->write(
    opencr_control_table.cmd_velocity_linear_x.address, 24, p_data);

  return comm_result;
}

uint8_t OpenCR::read_byte(const uint16_t & address)
{
  return dxl_sdk_wrapper_->read_byte(address);
}

void OpenCR::write_byte(const uint16_t & address, uint8_t data)
{
  dxl_sdk_wrapper_->write_byte(address, data);
}

void OpenCR::send_heartbeat(const uint8_t & count)
{
  this->write_byte(opencr_control_table.heartbeat.address, count);
}
}  // namespace turtlebot3_hardware
}  // namespace robotis
