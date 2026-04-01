#include <cmath>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "stream/stream.h"
#include "stream/raw_stream.h"

namespace beidou {

RawStream::RawStream(const rclcpp::Node::SharedPtr& node, const std::string& config_yaml)
  : node_(node), config_yaml_(config_yaml) { }

RawStream::~RawStream() {
  //停止dataspin循环
  is_running_ = false;
  if (data_parser_thread_ != nullptr && data_parser_thread_->joinable()) {
    data_parser_thread_->join();
  }

  //停止串口流serial stream
  Disconnect();
}

bool RawStream::Init() {

  //1.创建数据处理变量并初始化，串口读取数据后，将数据移交给data_parser
  data_parser_ = std::make_unique<beidou::DataParser>(node_);
  if (!data_parser_->Init()) {
    std::cout << "init data parser failed.\n";
    return false;
  }

  //2.读取串口配置文件，并创建串口流与串口流状态变量
  //2.1加载yaml,读取device_name和baud_rate
  std::string device_name = "";
  int baud_rate = 0;
  std::cout << "load yaml from " << config_yaml_ << "\n";
  
  auto yaml = YAML::LoadFile(config_yaml_);
  try {
    device_name = yaml["device_name"].as<std::string>();
    baud_rate = yaml["baud_rate"].as<int>();
  } catch (...) {
    std::cout << "failed to parse yaml\n";
    return false;
  }

  //2.2创建串口流 serial_stream
  serial_stream_.reset(Stream::create_serial(device_name.c_str(), baud_rate));
  if (serial_stream_ == nullptr) {
    std::cout << "failed to create data stream.\n";
    return false;
  }

  //2.3创建串口流状态 serial_stream_status
  serial_stream_status_.reset(new Status());
  if (serial_stream_status_ == nullptr ) {
    std::cout << "failed to create data stream status.\n";
    return false;
  }
  
  //2.4链接串口
  if (!Connect()) {
    std::cout << "beidou ins driver connect failed.\n";
    return false;
  }

  return true;
}

void RawStream::Run() {
  is_running_ = true;
  data_parser_thread_ = std::make_unique<std::thread>(std::thread(&RawStream::DataSpin, this));
}

bool RawStream::Connect() {
  if (serial_stream_ == nullptr) return false;

  if (serial_stream_->get_status() != Stream::Status::CONNECTED) {
    if (!serial_stream_->Connect()) {
      std::cout << "serial stream connect failed.\n";
      return false;
    }

    serial_stream_status_->status = Stream::Status::CONNECTED;
  }

  return true;
}

bool RawStream::Disconnect() {
  if (serial_stream_ == nullptr) return false;
  
  if (serial_stream_->get_status() == Stream::Status::CONNECTED) {
    if (!serial_stream_->Disconnect()) {
      std::cerr << "serial stream disconnect failed.\n";
      return false;
    }

    serial_stream_status_->status = Stream::Status::DISCONNECTED;
  }

  return true;
}

void RawStream::DataSpin() {
  //循环读取数据，并将数据交给data_parser处理
  //data_parser将数据交给novatel_parser处理，并使用ros发布结果
  while (is_running_&&rclcpp::ok()) {
    size_t length = serial_stream_->read(buffer_, READ_SIZE);
    if (length > 0) {
      data_parser_->ParseRawData(buffer_, length);
    }
  }
}
} // namesapce beidou