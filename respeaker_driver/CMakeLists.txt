cmake_minimum_required(VERSION 3.8)
project(respeaker_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robohead_interfaces REQUIRED)  

# PortAudio
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_executable(main src/main.cpp)

# Зависимости ROS2
ament_target_dependencies(main
  rclcpp
  std_msgs
  robohead_interfaces
)

#линковка PortAudio
target_include_directories(main PRIVATE ${PORTAUDIO_INCLUDE_DIRS})
target_link_libraries(main ${PORTAUDIO_LIBRARIES})
target_compile_options(main PRIVATE ${PORTAUDIO_CFLAGS_OTHER})

target_include_directories(main PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(main ${LIBUSB_LIBRARIES})

target_compile_features(main PUBLIC c_std_99 cxx_std_17)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()