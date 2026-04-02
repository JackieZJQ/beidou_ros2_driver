#pragma once

#include <memory>
#include <string>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <builtin_interfaces/msg/time.hpp>

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

  rclcpp::Time ToRosTimeFromGpsTime(uint32_t gps_week, double gps_seconds);
  void EstimateGpsUtcDiffSeconds(uint32_t gps_week, double gps_seconds);
  void InitLeapSeconds(uint32_t gps_week, double gps_seconds);

private:
  bool init_flag_ = false;
  std::unique_ptr<Parser> novatel_parser_;

  // corrimudata的频率，单位Hz
  double corrimudata_hz_ = 100.f;

  // GPS时间 转 UTC时间参数
  double leap_seconds_ = 18.0;
  bool leap_seconds_initialized_ = false;
  size_t diff_window_size_ = 100;
  std::deque<double> diff_window_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<beidou_ins_driver::msg::Inspva>::SharedPtr inspva_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_publisher_;

};
}  // namespace beidou

