cmake_minimum_required(VERSION 3.8)
project(bt_app_2025)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(opennav_docking_msgs REQUIRED)

# Include directories
include_directories(
  include
)

add_executable(bt_m src/bt_m.cpp)
target_link_libraries(bt_m
  BTNODE_RECEIVER
  BTNODE_FIRMWARE
  BTNODE_NAVIGATION
  BTNODE_OTHERS
  BTNODE_UTIL
  jsoncpp
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(bt_m
  ${THIS_PACKAGE_DEPS}
  "behaviortree_ros2"
  "behaviortree_cpp"
  "btcpp_ros2_interfaces"
  "nav2_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_srvs"
  "std_msgs"
  "geometry_msgs"
  "jsoncpp"
)
install(TARGETS
  bt_m
  DESTINATION lib/${PROJECT_NAME}
)

add_library(BTNODE_RECEIVER src/bt_nodes_receiver.cpp)
target_link_libraries(BTNODE_RECEIVER
  BTNODE_UTIL
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BTNODE_RECEIVER
  "behaviortree_ros2"
  "behaviortree_cpp"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_DEPS}
)

add_library(BTNODE_FIRMWARE src/bt_nodes_firmware.cpp)
target_link_libraries(BTNODE_FIRMWARE
  BTNODE_UTIL
  BTNODE_RECEIVER
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BTNODE_FIRMWARE
  "btcpp_ros2_interfaces"
  "nav2_msgs"
  "behaviortree_ros2"
  "behaviortree_cpp"
  "std_msgs"
  "std_srvs"
  "tf2_ros"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_DEPS}
)

add_library(BTNODE_NAVIGATION src/bt_nodes_navigation.cpp)
target_link_libraries(BTNODE_NAVIGATION
  BTNODE_UTIL
  BTNODE_RECEIVER
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BTNODE_NAVIGATION
  "btcpp_ros2_interfaces"
  "opennav_docking_msgs"
  "nav2_msgs"
  "behaviortree_ros2"
  "behaviortree_cpp"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_DEPS}
)

add_library(BTNODE_OTHERS src/bt_nodes_others.cpp)
target_link_libraries(BTNODE_OTHERS
  BTNODE_UTIL
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BTNODE_OTHERS
  "behaviortree_ros2"
  "behaviortree_cpp"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_DEPS}
)

add_library(BTNODE_UTIL src/bt_nodes_util.cpp)
target_link_libraries(BTNODE_UTIL
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BTNODE_UTIL
  "behaviortree_cpp"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "rclcpp"
  ${THIS_PACKAGE_DEPS}
)

install(DIRECTORY
  bt_m_config
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(behaviortree_ros2)
ament_export_dependencies(behaviortree_cpp)

ament_package()
