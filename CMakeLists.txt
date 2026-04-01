cmake_minimum_required(VERSION 3.9)
project(beidou_ins_driver)

SET(CMAKE_BUILD_TYPE "Release")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

#自定义消息
##################################################
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Inspva.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
##################################################

include_directories(${PROJECT_SOURCE_DIR}
                    ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}_lib SHARED 
            parser/data_parser.cc 
            parser/novatel_parser.cc
            stream/raw_stream.cc 
            stream/serial_stream.cc)   

#beidou_stream_node
##################################################
add_executable(beidou_stream_node beidou_stream_node.cc)
target_link_libraries(beidou_stream_node ${PROJECT_NAME}_lib yaml-cpp)
ament_target_dependencies(beidou_stream_node rclcpp ament_index_cpp std_msgs sensor_msgs yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp std_msgs sensor_msgs yaml-cpp)
rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
##################################################

#install
##################################################
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS beidou_stream_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME})
##################################################
ament_package()
