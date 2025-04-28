cmake_minimum_required(VERSION 3.8)
project(rival_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

# Include directories
include_directories(
  include
)

add_executable(rival_main src/rival_main.cpp)
target_link_libraries(rival_main
  BTNODE_NAVIGATION
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(rival_main
  ${THIS_PACKAGE_DEPS}
  "behaviortree_ros2"
  "behaviortree_cpp"
  "rclcpp"
)
install(TARGETS
  rival_main
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(game src/game.cpp)
# target_link_libraries(game
#   BTNODE_NAVIGATION
#   ${ARMADILLO_LIBRARIES})
# ament_target_dependencies(game
#   ${THIS_PACKAGE_DEPS}
#   "behaviortree_ros2"
#   "behaviortree_cpp"
#   "rclcpp"
#   "geometry_msgs"
# )
# install(TARGETS
#   game
#   DESTINATION lib/${PROJECT_NAME}
# )

add_library(BTNODE_NAVIGATION src/rival_navigation.cpp)
target_link_libraries(BTNODE_NAVIGATION  
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BTNODE_NAVIGATION
  "nav_msgs"
  "behaviortree_ros2"
  "behaviortree_cpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "rclcpp"
  ${THIS_PACKAGE_DEPS}
)

install(DIRECTORY
  rival_main_config
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(behaviortree_ros2)
ament_export_dependencies(behaviortree_cpp)

ament_package()
