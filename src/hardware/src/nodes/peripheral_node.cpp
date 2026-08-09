// Copyright 2026 Thecnfor
// SPDX-License-Identifier: Proprietary
//
// peripheral_node — 外设短操作服务 (middle layer 2026-08-09).
//
// Spec: docs/superpowers/specs/2026-08-09-midlayer-ros2control-cartesian-design.md §8
//
// Thin service shell over MC602Adapter (bridge transport): beep / RGB LED /
// dot-matrix text / nixie / digital output. These are SHORT, cosmetic or
// one-shot operations — services (not actions) are the right interface.
//
//   /rak/hw/peripheral/beep        SetBeep.srv
//   /rak/hw/peripheral/rgb_led     SetRgbLed.srv
//   /rak/hw/peripheral/led_show    LedShow.srv
//   /rak/hw/peripheral/nixie       SetNixie.srv
//   /rak/hw/peripheral/digital_out SetDigitalOut.srv

#include "hardware/mc602_adapter.hpp"
#include "hardware/transport_factory.hpp"

#include <msgs/srv/set_beep.hpp>
#include <msgs/srv/set_rgb_led.hpp>
#include <msgs/srv/led_show.hpp>
#include <msgs/srv/set_nixie.hpp>
#include <msgs/srv/set_digital_out.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace hardware
{

class PeripheralNode : public rclcpp::Node
{
public:
  explicit PeripheralNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("peripheral_node", options)
  {
    this->declare_parameter<std::string>("mc602_serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("mc602_baud", 1000000);
    this->declare_parameter<std::string>("mc602_transport", "bridge");

    const std::string port = this->get_parameter("mc602_serial_port").as_string();
    const int baud = this->get_parameter("mc602_baud").as_int();
    const std::string mode = this->get_parameter("mc602_transport").as_string();

    adapter_ = std::make_unique<MC602Adapter>(
      make_mc602_transport(this, mode, port, static_cast<uint32_t>(baud)));
    adapter_->open();
    RCLCPP_INFO(this->get_logger(), "peripheral_node up via %s transport", mode.c_str());

    srv_beep_ = this->create_service<msgs::srv::SetBeep>(
      "/rak/hw/peripheral/beep",
      [this](const std::shared_ptr<msgs::srv::SetBeep::Request> req,
             std::shared_ptr<msgs::srv::SetBeep::Response> resp) {
        try {
          adapter_->beep(req->freq, static_cast<float>(req->duration_ms) / 1000.0f);
          resp->ok = true;
        } catch (const std::exception & e) {
          resp->ok = false;
          resp->error = e.what();
        }
      });

    srv_led_ = this->create_service<msgs::srv::SetRgbLed>(
      "/rak/hw/peripheral/rgb_led",
      [this](const std::shared_ptr<msgs::srv::SetRgbLed::Request> req,
             std::shared_ptr<msgs::srv::SetRgbLed::Response> resp) {
        try {
          adapter_->set_led_light(static_cast<uint8_t>(req->led_id), req->r, req->g, req->b);
          resp->ok = true;
        } catch (const std::exception & e) {
          resp->ok = false;
          resp->error = e.what();
        }
      });

    srv_show_ = this->create_service<msgs::srv::LedShow>(
      "/rak/hw/peripheral/led_show",
      [this](const std::shared_ptr<msgs::srv::LedShow::Request> req,
             std::shared_ptr<msgs::srv::LedShow::Response> resp) {
        try {
          adapter_->set_led_show(req->text);
          resp->ok = true;
        } catch (const std::exception & e) {
          resp->ok = false;
          resp->error = e.what();
        }
      });

    srv_nixie_ = this->create_service<msgs::srv::SetNixie>(
      "/rak/hw/peripheral/nixie",
      [this](const std::shared_ptr<msgs::srv::SetNixie::Request> req,
             std::shared_ptr<msgs::srv::SetNixie::Response> resp) {
        try {
          adapter_->set_nixie(req->value);
          resp->ok = true;
        } catch (const std::exception & e) {
          resp->ok = false;
          resp->error = e.what();
        }
      });

    srv_dout_ = this->create_service<msgs::srv::SetDigitalOut>(
      "/rak/hw/peripheral/digital_out",
      [this](const std::shared_ptr<msgs::srv::SetDigitalOut::Request> req,
             std::shared_ptr<msgs::srv::SetDigitalOut::Response> resp) {
        try {
          adapter_->set_dout(static_cast<uint8_t>(req->port), req->on ? 1 : 0);
          resp->ok = true;
        } catch (const std::exception & e) {
          resp->ok = false;
          resp->error = e.what();
        }
      });

    RCLCPP_INFO(this->get_logger(),
      "peripheral services: beep / rgb_led / led_show / nixie / digital_out");
  }

  ~PeripheralNode() override
  {
    try {
      if (adapter_) {
        adapter_->close();
      }
    } catch (...) {}
  }

private:
  std::unique_ptr<MC602Adapter> adapter_;
  rclcpp::Service<msgs::srv::SetBeep>::SharedPtr srv_beep_;
  rclcpp::Service<msgs::srv::SetRgbLed>::SharedPtr srv_led_;
  rclcpp::Service<msgs::srv::LedShow>::SharedPtr srv_show_;
  rclcpp::Service<msgs::srv::SetNixie>::SharedPtr srv_nixie_;
  rclcpp::Service<msgs::srv::SetDigitalOut>::SharedPtr srv_dout_;
};

}  // namespace hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hardware::PeripheralNode>());
  rclcpp::shutdown();
  return 0;
}
