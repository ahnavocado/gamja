cmake_minimum_required(VERSION 3.10)
project(pc2_preprocessor)

# C++ 표준
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pc2_preprocessor_node
  src/pc2_preprocessor.cpp
)

ament_target_dependencies(pc2_preprocessor_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  pcl_conversions
)

target_link_libraries(pc2_preprocessor_node
  ${PCL_LIBRARIES}
)

install(TARGETS pc2_preprocessor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/
  DESTINATION include/)

ament_package()
