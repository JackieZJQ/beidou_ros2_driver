#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include <Eigen/Geometry>
#include <boost/array.hpp>

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

void DataParser::PublishInspva(const MessagePtr message) {
  novatel::InsPva* pva = static_cast<novatel::InsPva*>(message);
  beidou_ins_driver::msg::Inspva msg;
  //ros当前时间
  msg.header.stamp = rclcpp::Clock().now();

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

  //ros当前时间
  msg.header.stamp = rclcpp::Clock().now();

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

  msg.header.stamp = rclcpp::Clock().now(); //todo 转换为GPS时间，而不是解析消息的时间
  msg.linear_acceleration.x = imu->y_velocity_change * corrimudata_hz_;
  msg.linear_acceleration.y = -imu->x_velocity_change * corrimudata_hz_;
  msg.linear_acceleration.z = imu->z_velocity_change * corrimudata_hz_;
  msg.angular_velocity.x = imu->y_angle_change * corrimudata_hz_;
  msg.angular_velocity.y = -imu->x_angle_change * corrimudata_hz_;
  msg.angular_velocity.z = imu->z_angle_change * corrimudata_hz_;

  imu_publisher_->publish(msg);
}
}  // namespace beidou

