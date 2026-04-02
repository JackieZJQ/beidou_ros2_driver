#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include <Eigen/Geometry>
#include <boost/array.hpp>
#include <chrono>

#include "parser/data_parser.h"

namespace beidou {
DataParser::DataParser(const rclcpp::Node::SharedPtr& node)
  : node_(node) {}

bool DataParser::Init() {
  if (!InitParser()) {
    std::cout << "failed to init parser.\n";
    return false;
  }

  if (!InitPublisher()) {
    std::cout << "failed to init data publisher.\n";
    return false;
  }

  init_flag_ = true;
  return true;
}

bool DataParser::InitParser() {  
  novatel_parser_.reset(Parser::CreateParser());
  if (novatel_parser_ == nullptr) {
    std::cout << "failed to create data parser.\n";
    return false;
  }

  return true;
}

bool DataParser::InitPublisher() {
  imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("/beidou/corrimudata", 10);
  inspva_publisher_ = node_->create_publisher<beidou_ins_driver::msg::Inspva>("/beidou/inspva", 10);
  navsatfix_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("/beidou/navsatfix", 10);

  return true;
}

void DataParser::ParseRawData(const uint8_t* buffer_, size_t length) {
  if (!init_flag_) {
    std::cout << "data parser not init.\n";
    return;
  }
  
  //更新novatel_parser处理模块中buffer_起始点与终止点
  novatel_parser_->Update(buffer_, length);

  MessageInfo message;
  do {
    novatel_parser_->GetMessages(&message);
    DispatchMessage(message); 
  } while (message.type != MessageType::NONE);
}

void DataParser::DispatchMessage(const MessageInfo& message_info) {
  void* message = message_info.message_ptr;
  switch (message_info.type) {
    case MessageType::IMU:
      PublishCorrimu(message);
      break;
    case MessageType::INS:
      PublishInspva(message);
      PublishNavsatfix(message);
      break;
    default:
      break;
  }
}

rclcpp::Time DataParser::ToRosTimeFromGpsTime(uint32_t gps_week, double gps_seconds) {
  // utc = gps + epoch_diff - leap_seconds
  double unix_utc = static_cast<double>(gps_week) * 604800.0 + gps_seconds + 315964800.0 - static_cast<double>(leap_seconds_);

  int64_t ns = static_cast<int64_t>(std::llround(unix_utc * 1e9));
  return rclcpp::Time(ns, RCL_SYSTEM_TIME);
}

void DataParser::EstimateGpsUtcDiffSeconds(uint32_t gps_week, double gps_seconds) {

  // gps -> utc (未减润秒)
  const double gps_utc_like = static_cast<double>(gps_week) * 604800.0 + gps_seconds + 315964800.0;

  // 系统UTC时间，已使用GPRMC消息同时
  const double sys_utc = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count())*1e-9;

  // diff = GPS-UTC + 链路延迟项
  const double diff = gps_utc_like - sys_utc;

  diff_window_.push_back(diff);
  if (diff_window_.size() > diff_window_size_) diff_window_.pop_front();

  // 样本足够后锁定整数 leap_seconds
  if (!leap_seconds_initialized_ && diff_window_.size() >= 20) {

    std::sort(diff_window_.begin(), diff_window_.end());
    const size_t n = diff_window_.size();
    const double med =  (n % 2 == 0) ? (diff_window_[n / 2 - 1] + diff_window_[n / 2]) / 2.0 : diff_window_[n / 2];

    leap_seconds_ = static_cast<int>(std::llround(med)); // 你要的“直接取整”
    leap_seconds_initialized_ = true;
    std::cout << "[time_sync] lock leap_seconds=" << leap_seconds_
              << " (median diff=" << med << "s)\n";
  }

  return;
}

void DataParser::InitLeapSeconds(uint32_t gps_week, double gps_seconds) {
  if (leap_seconds_initialized_) return;

  EstimateGpsUtcDiffSeconds(gps_week, gps_seconds);

}

void DataParser::PublishInspva(const MessagePtr message) {
  novatel::InsPva* pva = static_cast<novatel::InsPva*>(message);
  beidou_ins_driver::msg::Inspva msg;
  
  //estimateleapseconds
  InitLeapSeconds(pva->gps_week, pva->gps_seconds);
  if (!leap_seconds_initialized_) return;

  msg.header.stamp = ToRosTimeFromGpsTime(pva->gps_week, pva->gps_seconds);

  //经纬高
  msg.latitude = pva->latitude;
  msg.longitude = pva->longitude;
  msg.height = pva->height;
  msg.north_velocity = pva->north_velocity;
  msg.east_velocity = pva->east_velocity;
  msg.up_velocity = pva->up_velocity;
  msg.roll = pva->roll;
  msg.pitch = pva->pitch;
  msg.azimuth = pva->azimuth;

  inspva_publisher_->publish(msg);
}

void DataParser::PublishNavsatfix(const MessagePtr message) {
  novatel::InsPva *pva = static_cast<novatel::InsPva *>(message);
  sensor_msgs::msg::NavSatFix msg;

  // utc时间
  InitLeapSeconds(pva->gps_week, pva->gps_seconds);
  if (!leap_seconds_initialized_) return;

  msg.header.stamp = ToRosTimeFromGpsTime(pva->gps_week, pva->gps_seconds);

  //经纬高
  msg.latitude = pva->latitude;
  msg.longitude = pva->longitude;
  msg.altitude = pva->height;

  //定位状态
  if (pva->status == novatel::InsStatus::SOLUTION_GOOD || pva->status == novatel::InsStatus::ALIGNMENT_COMPLETE) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }

  navsatfix_publisher_->publish(msg);
}

void DataParser::PublishCorrimu(const MessagePtr message) {
  novatel::CorrImuData* imu  = static_cast<novatel::CorrImuData*>(message);
  sensor_msgs::msg::Imu msg;

  // utc时间
  InitLeapSeconds(imu->gps_week, imu->gps_seconds);
  if (!leap_seconds_initialized_) return;

  msg.header.stamp = ToRosTimeFromGpsTime(imu->gps_week, imu->gps_seconds);
  
  msg.linear_acceleration.x = imu->y_velocity_change * corrimudata_hz_;
  msg.linear_acceleration.y = -imu->x_velocity_change * corrimudata_hz_;
  msg.linear_acceleration.z = imu->z_velocity_change * corrimudata_hz_;
  msg.angular_velocity.x = imu->y_angle_change * corrimudata_hz_;
  msg.angular_velocity.y = -imu->x_angle_change * corrimudata_hz_;
  msg.angular_velocity.z = imu->z_angle_change * corrimudata_hz_;

  imu_publisher_->publish(msg);
}
}  // namespace beidou

