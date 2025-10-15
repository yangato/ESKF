cmake_minimum_required(VERSION 3.28)
project(eskf)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(cmake_modules)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  include/eskf
  ${EIGEN3_INCLUDE_DIRS}
)

# include headers
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${mavros_INCLUDE_DIRS}
  include
  include/eskf
)

# ---- Core filter library (your ESKF) ----
add_library(ESKF SHARED
  src/ESKF.cpp
)
target_include_directories(ESKF PUBLIC
  include
  include/eskf
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(ESKF
  rclcpp
)

# ---- Node library (ROS 2 node class) ----
add_library(eskf_node_lib SHARED
  src/Node.cpp
)
target_include_directories(eskf_node_lib PUBLIC
  include
  include/eskf
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(eskf_node_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)
target_link_libraries(eskf_node_lib
  ESKF
)

# ---- Executable (contains main()) ----
# Keep src/eskf.cpp as your entry point; it should create rclcpp::Node/Spin.
add_executable(eskf_node
  src/eskf.cpp
)
ament_target_dependencies(eskf_node
  rclcpp
)
target_link_libraries(eskf_node
  eskf_node_lib
)

# ---- Install ----
install(TARGETS
  ESKF
  eskf_node_lib
  eskf_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# ---- Export for downstream packages ----
ament_export_include_directories(include include/eskf)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  Eigen3
)
# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
ament_package()