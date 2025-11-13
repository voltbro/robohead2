cmake_minimum_required(VERSION 3.10)
project(media_driver LANGUAGES CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robohead_interfaces REQUIRED)

# Find libmpv via pkg-config if available, otherwise try to find library
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

include_directories(
  include
)

add_executable(main src/main.cpp)
#target_include_directories(main ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(main ${MPV_LIBS})
ament_target_dependencies(main  rclcpp robohead_interfaces)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
