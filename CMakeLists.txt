cmake_minimum_required(VERSION 3.8)

project(sick_visionary_ros
        VERSION 1.1.2
        LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find package for header message type
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost 1.41 REQUIRED COMPONENTS thread system)

### BUILD ###
include_directories(
  include
  sick_visionary_cpp_shared
)

add_subdirectory(sick_visionary_cpp_shared)

add_executable(sick_visionary_t_mini_node src/visionary_t_mini.cpp)
## ignore visionary_s for ROS2 port

ament_target_dependencies(sick_visionary_t_mini_node
  rclcpp
  sensor_msgs
  std_msgs
  builtin_interfaces
  image_transport
  cv_bridge
  diagnostic_updater
  OpenCV)
# link against the local shared library built in sick_visionary_cpp_shared
target_link_libraries(sick_visionary_t_mini_node
  sick_visionary_cpp_shared
  Threads::Threads
  Boost::thread
  Boost::system
)

## skip visionary_s

### INSTALL ###
# install executable
install(TARGETS sick_visionary_t_mini_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES launch/sick_visionary-t_mini.launch
  DESTINATION share/${PROJECT_NAME}/launch
  RENAME sick_visionary-t_mini.launch.xml
)

# ament package directive
ament_package()
