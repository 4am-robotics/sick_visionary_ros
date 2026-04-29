cmake_minimum_required(VERSION 3.8)

project(sick_visionary_ros
        VERSION 1.1.2
        LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(image_transport REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV REQUIRED)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

### BUILD ###
include_directories(
  include
  sick_visionary_cpp_shared
)

add_subdirectory(sick_visionary_cpp_shared)

add_executable(sick_visionary_t_mini_node src/visionary_t_mini.cpp)
# sick_visionary_s_node intentionally not built in this fork.

ament_target_dependencies(sick_visionary_t_mini_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  diagnostic_updater
  image_transport
  OpenCV
)

target_link_libraries(sick_visionary_t_mini_node
  sick_visionary_cpp_shared
  ${OpenCV_LIBS}
  Boost::thread
  Boost::system
)

### INSTALL ###
install(TARGETS sick_visionary_t_mini_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
