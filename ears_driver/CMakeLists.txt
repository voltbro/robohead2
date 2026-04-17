cmake_minimum_required(VERSION 3.8)
project(ears_driver)

# Set version of compiler
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set level of warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robohead_interfaces REQUIRED)

add_executable(${PROJECT_NAME}_node src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp robohead_interfaces)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(servo_pulse_check src/servo_pulse_check.cpp)

install(TARGETS
  servo_pulse_check
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()