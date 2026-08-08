// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// SystemIoNode — 非 arm/chassis 杂项设备的类型化服务门面。
// Spec: docs/superpowers/specs/2026-08-09-ros2-layering-interfaces-design.md §6
//
// 设备: beep / led_light / led_show / nixie / board_key / bluetooth_pad /
// 通用传感器按需读。一律经 mc602_bridge(单串口所有者)。
// 纯逻辑在 SystemIo 类(src/system_io.cpp),本文件只是 ROS2 shell。

#include "hardware/mc602_adapter.hpp"
#include "hardware/system_io.hpp"
#include "hardware/transport_factory.hpp"

#include <msgs/srv/beep.hpp>
#include <msgs/srv/led_show.hpp>
#include <msgs/srv/nixie.hpp>
#include <msgs/srv/read_int_array.hpp>
#include <msgs/srv/sensor_query.hpp>
#include <msgs/srv/set_rgb_led.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace vw = hardware;

class SystemIoNode : public rclcpp::Node
{
public:
  explicit SystemIoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("system_io_node", options)
  {
    this->declare_parameter<std::string>("mc602_serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("mc602_baud", 115200);
    this->declare_parameter<std::string>("mc602_transport", "bridge");

    const std::string port = this->get_parameter("mc602_serial_port").as_string();
    const int baud = this->get_parameter("mc602_baud").as_int();
    const std::string mode = this->get_parameter("mc602_transport").as_string();

    adapter_ = std::make_unique<vw::MC602Adapter>(
      vw::make_mc602_transport(this, mode, port, static_cast<uint32_t>(baud)));
    adapter_->open();
    io_ = std::make_unique<vw::SystemIo>(adapter_.get());

    srv_read_sensor_ = this->create_service<msgs::srv::SensorQuery>(
      "/rak/hw/system/read_sensor",
      [this](const std::shared_ptr<msgs::srv::SensorQuery::Request> req,
             std::shared_ptr<msgs::srv::SensorQuery::Response> resp) {
        io_->read_sensor(req->port, req->type, resp->ok, resp->error, resp->value);
      });

    srv_beep_ = this->create_service<msgs::srv::Beep>(
      "/rak/hw/system/beep",
      [this](const std::shared_ptr<msgs::srv::Beep::Request> req,
             std::shared_ptr<msgs::srv::Beep::Response> resp) {
        io_->beep(req->freq, req->duration_s, resp->ok, resp->error);
      });

    srv_led_light_ = this->create_service<msgs::srv::SetRgbLed>(
      "/rak/hw/system/led_light",
      [this](const std::shared_ptr<msgs::srv::SetRgbLed::Request> req,
             std::shared_ptr<msgs::srv::SetRgbLed::Response> resp) {
        io_->set_led_light(req->led_id, req->r, req->g, req->b,
                           resp->ok, resp->error);
      });

    srv_led_show_ = this->create_service<msgs::srv::LedShow>(
      "/rak/hw/system/led_show",
      [this](const std::shared_ptr<msgs::srv::LedShow::Request> req,
             std::shared_ptr<msgs::srv::LedShow::Response> resp) {
        io_->set_led_show(req->text, resp->ok, resp->error);
      });

    srv_nixie_ = this->create_service<msgs::srv::Nixie>(
      "/rak/hw/system/nixie",
      [this](const std::shared_ptr<msgs::srv::Nixie::Request> req,
             std::shared_ptr<msgs::srv::Nixie::Response> resp) {
        io_->set_nixie(req->value, resp->ok, resp->error);
      });

    srv_read_key_ = this->create_service<msgs::srv::ReadIntArray>(
      "/rak/hw/system/read_key",
      [this](const std::shared_ptr<msgs::srv::ReadIntArray::Request>,
             std::shared_ptr<msgs::srv::ReadIntArray::Response> resp) {
        io_->read_key(resp->ok, resp->error, resp->values);
      });

    srv_read_pad_ = this->create_service<msgs::srv::ReadIntArray>(
      "/rak/hw/system/read_pad",
      [this](const std::shared_ptr<msgs::srv::ReadIntArray::Request>,
             std::shared_ptr<msgs::srv::ReadIntArray::Response> resp) {
        io_->read_pad(resp->ok, resp->error, resp->values);
      });

    RCLCPP_INFO(this->get_logger(),
      "SystemIoNode up: %s @ %d baud via %s", port.c_str(), baud, mode.c_str());
  }

  ~SystemIoNode() override
  {
    try {
      if (adapter_) {
        adapter_->close();
      }
    } catch (...) {
      // destructor must not throw
    }
  }

private:
  std::unique_ptr<vw::MC602Adapter> adapter_;
  std::unique_ptr<vw::SystemIo> io_;

  rclcpp::Service<msgs::srv::SensorQuery>::SharedPtr srv_read_sensor_;
  rclcpp::Service<msgs::srv::Beep>::SharedPtr srv_beep_;
  rclcpp::Service<msgs::srv::SetRgbLed>::SharedPtr srv_led_light_;
  rclcpp::Service<msgs::srv::LedShow>::SharedPtr srv_led_show_;
  rclcpp::Service<msgs::srv::Nixie>::SharedPtr srv_nixie_;
  rclcpp::Service<msgs::srv::ReadIntArray>::SharedPtr srv_read_key_;
  rclcpp::Service<msgs::srv::ReadIntArray>::SharedPtr srv_read_pad_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded: bridge mode blocks the callback on a service round-trip.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(std::make_shared<SystemIoNode>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
