cmake_minimum_required(VERSION 3.10)
project(pc2_preprocessor)

# C++ standard and compile options (aligned with style)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Dependencies =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)

# PCL includes/definitions (use include_directories style like orbslam3)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}/include
)
link_directories(
  include
)

add_definitions(${PCL_DEFINITIONS})

# ===== Executables =====
add_executable(pc2_preprocessor_node
  src/pc2_preprocessor.cpp
)
ament_target_dependencies(pc2_preprocessor_node
  rclcpp sensor_msgs nav_msgs std_msgs pcl_conversions
)
target_link_libraries(pc2_preprocessor_node
  ${PCL_LIBRARIES}
)

add_executable(dummy_cloud_pub
  src/dummy_cloud_pub.cpp
)
ament_target_dependencies(dummy_cloud_pub
  rclcpp sensor_msgs pcl_conversions
)
target_link_libraries(dummy_cloud_pub
  ${PCL_LIBRARIES}
)

# ===== Install rules =====
install(TARGETS pc2_preprocessor_node dummy_cloud_pub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ament index/hooks helper: install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ===== Export (help overlay recognition/reuse) =====
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp sensor_msgs nav_msgs std_msgs pcl_conversions PCL
)

# Must be last
ament_package()
