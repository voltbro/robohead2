cmake_minimum_required(VERSION 3.8)
project(ears_driver)  # ← ВАЖНО: должно совпадать с именем папки пакета!

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robohead_interfaces REQUIRED)

add_executable(main src/main.cpp)

ament_target_dependencies(main
  rclcpp
  robohead_interfaces
)

# Линковка с rt для usleep
target_link_libraries(main rt)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()