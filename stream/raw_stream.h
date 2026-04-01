#pragma once

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "stream/stream.h"
#include "parser/data_parser.h"

namespace beidou {

class RawStream {
public:
  RawStream(const rclcpp::Node::SharedPtr& node, const std::string& config_yaml);
  RawStream() = delete;

  ~RawStream();

  bool Init();

  struct Status {
    bool filter[Stream::NUM_STATUS] = {false};
    Stream::Status status;
  };

  void Run();

private:
  void DataSpin();
  bool Connect();
  bool Disconnect();

  static constexpr size_t BUFFER_SIZE = 2048;
  static constexpr size_t READ_SIZE = 500;
  uint8_t buffer_[BUFFER_SIZE] = {0};

  std::shared_ptr<Stream> serial_stream_;
  std::shared_ptr<Status> serial_stream_status_;

  std::unique_ptr<std::thread> data_parser_thread_;
  std::unique_ptr<DataParser> data_parser_;
  std::atomic<bool> is_running_;

  std::string config_yaml_;
  rclcpp::Node::SharedPtr node_;
};
} // namespace beidou