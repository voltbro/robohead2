cmake_minimum_required(VERSION 3.10)
project(media_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robohead_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find libmpv
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(MPV REQUIRED mpv)
  include_directories(${MPV_INCLUDE_DIRS})
  link_directories(${MPV_LIBRARY_DIRS})
  add_definitions(${MPV_CFLAGS_OTHER})
  set(MPV_LIBS ${MPV_LIBRARIES})
else()
  find_path(MPV_INCLUDE_DIR mpv/client.h)
  find_library(MPV_LIB mpv)
  include_directories(${MPV_INCLUDE_DIR})
  set(MPV_LIBS ${MPV_LIB})
endif()

# --- Важно: сюда добавляем include из cv_bridge и OpenCV ---
include_directories(
  include
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# --- Исполняемый файл ---
add_executable(main src/main.cpp)

target_link_libraries(main
  ${MPV_LIBS}
  ${OpenCV_LIBS}
)

ament_target_dependencies(main
  rclcpp
  robohead_interfaces
  sensor_msgs
  cv_bridge
)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
