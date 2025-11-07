cmake_minimum_required(VERSION 3.8)
project(media_driver)

# Требуем C++17
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robohead_interfaces REQUIRED)  

find_package(CURL REQUIRED)




# Исполняемый файл
add_executable(main src/main.cpp)

# Зависимости
ament_target_dependencies(main
  rclcpp
  std_msgs
  robohead_interfaces
)
target_link_libraries(main CURL::libcurl)
# Установка
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()