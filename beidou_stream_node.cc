#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "stream/raw_stream.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("beidou_stream_node");

  std::string package_path = ament_index_cpp::get_package_share_directory("beidou_ins_driver");
  std::string yaml_path = package_path + "/config/serial_stream.yaml";
  beidou::RawStream raw_stream(node, yaml_path);
  if (!raw_stream.Init()) std::cout << "init raw_stream failed...\n";
  raw_stream.Run();
  
  rclcpp::spin(node);
  rclcpp::shutdown(); 

  return 0; 
}
   


