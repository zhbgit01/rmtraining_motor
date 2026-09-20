cmake_minimum_required(VERSION 3.8)

project(motor_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(motor_simulator
  src/motor_simulator.cpp
)

ament_target_dependencies(motor_simulator
  rclcpp
  std_msgs
)

add_executable(input_source
  src/input_source.cpp
)

ament_target_dependencies(input_source
  rclcpp
  std_msgs
)

add_executable(pid_controller
  src/pid_controller.cpp
)

ament_target_dependencies(pid_controller
  rclcpp
  std_msgs
)

add_executable(dual_pid_controller
  src/dual_pid_controller.cpp
)

ament_target_dependencies(dual_pid_controller
  rclcpp
  std_msgs
)

install(TARGETS
  motor_simulator
  input_source
  pid_controller
  dual_pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()