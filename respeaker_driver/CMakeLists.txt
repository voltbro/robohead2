cmake_minimum_required(VERSION 3.8)
project(respeaker_driver)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS2 dependencies ──
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robohead_interfaces REQUIRED)

# ── System dependencies ──
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED IMPORTED_TARGET portaudio-2.0)
pkg_check_modules(LIBUSB    REQUIRED IMPORTED_TARGET libusb-1.0)
pkg_check_modules(ALSA      REQUIRED IMPORTED_TARGET alsa)

# ── C++ executable ──
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/audio_handler.cpp
  src/usb_handler.cpp
  src/usb_handler_xvf3000.cpp
  src/usb_handler_xvf3800.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  robohead_interfaces
)

target_link_libraries(${PROJECT_NAME}_node
  PkgConfig::PORTAUDIO
  PkgConfig::LIBUSB
  PkgConfig::ALSA
)

# ── Python scripts ──
install(PROGRAMS
  scripts/audio_to_wav.py
  scripts/echo_player.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install C++ targets ──
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install config & launch ──
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()