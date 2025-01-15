cmake_minimum_required(VERSION 3.8)
project(bt_app)

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
find_package(btcpp_ros2_interfaces REQUIRED)

# Include directories
include_directories(
  include
)

add_executable(bt_ros2 src/node/bt_ros2.cpp)
ament_target_dependencies(bt_ros2
  ${THIS_PACKAGE_DEPS}
  "behaviortree_ros2"
  "behaviortree_cpp"
  "btcpp_ros2_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
)
target_link_libraries(bt_ros2
  BT_ACTION_NODE 
  BT_DECORATOR_NODE 
  BT_UTILS_NODE_NODE 
)
install(TARGETS
  bt_ros2
  DESTINATION lib/${PROJECT_NAME}
)

add_library(navigationServer SHARED
  src/node/navigationServer.cpp)
target_include_directories(navigationServer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(navigationServer
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(navigationServer
  "btcpp_ros2_interfaces"
  "behaviortree_ros2"
  "behaviortree_cpp"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(navigationServer PLUGIN "bt_app::TestNavServer" EXECUTABLE test_nav_server)
install(TARGETS
  navigationServer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(localizationServer SHARED
  src/node/localizationServer.cpp)
target_include_directories(localizationServer PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(localizationServer
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(localizationServer
  "btcpp_ros2_interfaces"
  "behaviortree_ros2"
  "behaviortree_cpp"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(localizationServer PLUGIN "bt_app::TestLocServer" EXECUTABLE test_loc_server)

add_library(BT_ACTION_NODE src/bt_action_node_lib.cpp)
target_link_libraries(BT_ACTION_NODE 
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BT_ACTION_NODE 
  "btcpp_ros2_interfaces"
  "behaviortree_ros2"
  "behaviortree_cpp"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_DEPS})

add_library(BT_DECORATOR_NODE src/bt_decorator_node_lib.cpp)
target_link_libraries(BT_DECORATOR_NODE 
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BT_DECORATOR_NODE 
  "btcpp_ros2_interfaces"
  "behaviortree_ros2"
  "behaviortree_cpp"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_DEPS})

add_library(BT_UTILS_NODE_NODE src/bt_utils_node_lib.cpp)
target_link_libraries(BT_UTILS_NODE_NODE 
  ${ARMADILLO_LIBRARIES})
ament_target_dependencies(BT_UTILS_NODE_NODE 
  "btcpp_ros2_interfaces"
  "behaviortree_ros2"
  "behaviortree_cpp"
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_DEPS})

install(TARGETS
  navigationServer
  localizationServer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  bt_config
  # launch
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
