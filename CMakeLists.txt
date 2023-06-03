cmake_minimum_required(VERSION 3.8)
project(edu_robocup_rescue_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(edu_robocup_rescue_stack_node src/edu_robocup_rescue_stack_node.cpp src/localistionControlNode.cpp src/control.cpp src/localisation.cpp src/navigation.cpp src/loadMap.cpp)

ament_target_dependencies(edu_robocup_rescue_stack_node rclcpp geometry_msgs nav_msgs nav2_msgs std_msgs sensor_msgs lifecycle_msgs rclcpp_lifecycle)

target_include_directories(edu_robocup_rescue_stack_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  

target_compile_features(edu_robocup_rescue_stack_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS edu_robocup_rescue_stack_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch config_ekf params map   #Added config + map
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY keepout   #Added config + map
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
