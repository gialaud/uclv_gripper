cmake_minimum_required(VERSION 3.8)
project(uclv_robotiq_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uclv_robotiq REQUIRED)
find_package(uclv_modbus REQUIRED)
find_package(uclv_robotiq_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(robotiq_3f_gripper_node src/robotiq_3f_gripper_node.cpp)
# ament_target_dependencies(robotiq_3f_gripper_node rclcpp std_msgs uclv_robotiq uclv_robotiq_interfaces std_srvs)
add_executable(robotiq_2f_gripper_node src/robotiq_2f_gripper_node.cpp)
ament_target_dependencies(robotiq_2f_gripper_node rclcpp std_msgs uclv_robotiq uclv_robotiq_interfaces std_srvs uclv_modbus)

install(TARGETS
  # robotiq_3f_gripper_node 
  robotiq_2f_gripper_node 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
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
