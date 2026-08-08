// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SystemIo — see header.

#include "hardware/system_io.hpp"

#include <algorithm>
#include <exception>
#include <string>
#include <vector>

namespace hardware
{

namespace
{
// 与 msgs/srv/SensorQuery.srv + MC602Adapter::read_sensor dispatch 保持一致。
// 未知类型在本地拒绝,不打串口。
const std::vector<std::string> kSensorTypes = {
  "ir", "ultrasonic", "analog_input", "touch", "ambient_light", "encoder"};
}  // namespace

void SystemIo::read_sensor(uint8_t port, const std::string & type,
                           bool & ok, std::string & error, double & value)
{
  if (std::find(kSensorTypes.begin(), kSensorTypes.end(), type) ==
      kSensorTypes.end()) {
    ok = false;
    error = "unsupported sensor type '" + type + "'";
    return;
  }
  try {
    value = dev_->read_sensor(port, type);
    ok = true;
    error.clear();
  } catch (const std::exception & e) {
    ok = false;
    error = e.what();
  }
}

void SystemIo::beep(int freq, float duration_s, bool & ok, std::string & error)
{
  try {
    dev_->beep(freq, duration_s);
    ok = true;
    error.clear();
  } catch (const std::exception & e) {
    ok = false;
    error = e.what();
  }
}

void SystemIo::set_led_light(uint8_t led_id, int r, int g, int b,
                             bool & ok, std::string & error)
{
  try {
    dev_->set_led_light(led_id, r, g, b);
    ok = true;
    error.clear();
  } catch (const std::exception & e) {
    ok = false;
    error = e.what();
  }
}

void SystemIo::set_led_show(const std::string & text,
                            bool & ok, std::string & error)
{
  try {
    dev_->set_led_show(text);
    ok = true;
    error.clear();
  } catch (const std::exception & e) {
    ok = false;
    error = e.what();
  }
}

void SystemIo::set_nixie(int value, bool & ok, std::string & error)
{
  try {
    dev_->set_nixie(value);
    ok = true;
    error.clear();
  } catch (const std::exception & e) {
    ok = false;
    error = e.what();
  }
}

void SystemIo::read_key(bool & ok, std::string & error,
                        std::vector<int64_t> & values)
{
  try {
    values = dev_->read_board_key();
    ok = true;
    error.clear();
  } catch (const std::exception & e) {
    ok = false;
    error = e.what();
  }
}

void SystemIo::read_pad(bool & ok, std::string & error,
                        std::vector<int64_t> & values)
{
  try {
    values = dev_->read_bluetooth_pad();
    ok = true;
    error.clear();
  } catch (const std::exception & e) {
    ok = false;
    error = e.what();
  }
}

}  // namespace hardware
