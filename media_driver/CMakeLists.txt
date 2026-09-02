cmake_minimum_required(VERSION 3.10)
project(media_driver)

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

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robohead_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Find libmpv
find_package(PkgConfig REQUIRED)
pkg_check_modules(MPV REQUIRED IMPORTED_TARGET mpv)

# --- Важно: сюда добавляем include OpenCV ---
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Исходники
set(SOURCES
  src/mpv_player.cpp
  src/utils.cpp
  src/node.cpp
)
set(HEADERS 
  include/${PROJECT_NAME}/mpv_player.hpp
  include/${PROJECT_NAME}/utils.hpp
  include/${PROJECT_NAME}/node.hpp
  include/${PROJECT_NAME}/touch_publisher.hpp
)

# --- Исполняемый файл ---
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  ${SOURCES}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node
  # ${MPV_LIBS}
  PkgConfig::MPV
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  robohead_interfaces
  sensor_msgs
)

# ---
add_executable(touch_publisher_node
  src/touch_publisher.cpp
)
target_include_directories(touch_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(touch_publisher_node
  rclcpp
  robohead_interfaces
)


# Установка заголовочных файлов
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  touch_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
