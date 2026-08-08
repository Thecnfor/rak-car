// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SystemIo — 非 arm/chassis 杂项设备的类型化门面(纯类,可注入,无 ROS 依赖)。
//
// 节点 shell 只管把 7 个 service 请求翻译成这里的调用;本类负责:
//   1. 把 service 语义映射到 MC602AdapterIface(beep/led/nixie/key/pad/传感器)
//   2. 未知传感器 type 在本地拒绝(不打串口)
//   3. 驱动异常 → ok=false + error 文本

#pragma once

#include "hardware/mc602_adapter_iface.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace hardware
{

class SystemIo
{
public:
  explicit SystemIo(MC602AdapterIface * dev) : dev_(dev) {}

  void read_sensor(uint8_t port, const std::string & type,
                   bool & ok, std::string & error, double & value);
  void beep(int freq, float duration_s, bool & ok, std::string & error);
  void set_led_light(uint8_t led_id, int r, int g, int b,
                     bool & ok, std::string & error);
  void set_led_show(const std::string & text, bool & ok, std::string & error);
  void set_nixie(int value, bool & ok, std::string & error);
  void read_key(bool & ok, std::string & error, std::vector<int64_t> & values);
  void read_pad(bool & ok, std::string & error, std::vector<int64_t> & values);

private:
  MC602AdapterIface * dev_;
};

}  // namespace hardware
