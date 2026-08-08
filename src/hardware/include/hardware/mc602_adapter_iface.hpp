// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// MC602AdapterIface — SystemIo 依赖的窄设备接口(接口隔离)。
//
// MC602Adapter 同时实现 BaseController(底盘/臂的旧接口)与 MC602AdapterIface
// (杂项设备)。SystemIo 只依赖这个窄接口,测试注入假实现即可,不耦合整个驱动。

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace hardware
{

class MC602AdapterIface
{
public:
  virtual ~MC602AdapterIface() = default;

  // 通用传感器按需读;type 见 msgs/srv/SensorQuery.srv(与驱动 dispatch 一致):
  //   ir | ultrasonic | analog_input | touch | ambient_light | encoder
  virtual double read_sensor(uint8_t port, const std::string & type) = 0;

  virtual void beep(int freq, float duration_s) = 0;
  virtual void set_led_light(uint8_t led_id, int r, int g, int b) = 0;
  virtual void set_led_show(const std::string & text) = 0;
  virtual void set_nixie(int value) = 0;
  virtual std::vector<int64_t> read_board_key() = 0;
  virtual std::vector<int64_t> read_bluetooth_pad() = 0;
};

}  // namespace hardware
