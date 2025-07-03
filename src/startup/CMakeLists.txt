cmake_minimum_required(VERSION 3.8)
project(startup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

# Include directories
include_directories(
  include
)

add_executable(startup src/startup.cpp)
ament_target_dependencies(startup
  ${THIS_PACKAGE_DEPS}
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "btcpp_ros2_interfaces"
  "jsoncpp"
)
target_link_libraries(startup
  jsoncpp
)
install(TARGETS
  startup
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
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

ament_package()