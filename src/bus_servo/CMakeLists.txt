cmake_minimum_required(VERSION 3.5)
project(bus_servo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 (Ros2 default)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bus_servo_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)


add_executable(bus_servo_node src/bus_servo_node.cpp)
ament_target_dependencies(bus_servo_node rclcpp std_msgs diagnostic_msgs bus_servo_interfaces)

add_executable(bus_servo_config src/bus_servo_config.cpp)


install(TARGETS
  bus_servo_node
  bus_servo_config
  DESTINATION lib/${PROJECT_NAME})

ament_package()
