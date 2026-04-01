#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "parser/parser.h"
#include "parser/novatel_messages.h"
#include "beidou_ins_driver/msg/inspva.hpp"

namespace beidou {

class DataParser {
public:
  DataParser(const rclcpp::Node::SharedPtr& node);
  DataParser() = delete;

  void ParseRawData(const uint8_t* buffer_, size_t length);
  bool Init();

private:
  bool InitPublisher();
  bool InitParser();

  void DispatchMessage(const MessageInfo& message_info);
  void PublishInspva(const MessagePtr message);
  void PublishNavsatfix(const MessagePtr message);
  void PublishCorrimu(const MessagePtr message);

private:
  bool init_flag_ = false;
  std::unique_ptr<Parser> novatel_parser_;

  double corrimudata_hz_ = 100.f; //imu输出频率

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<beidou_ins_driver::msg::Inspva>::SharedPtr inspva_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_publisher_;

};
}  // namespace beidou

