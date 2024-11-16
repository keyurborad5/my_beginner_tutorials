cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
# execute_process(COMMAND cmake -E create_symlink
#                 "${PROJECT_BINARY_DIR}/compile_commands.json"
#                 "${PROJECT_SOURCE_DIR}/compile_commands.json")
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
)
# ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

add_executable(level2_integration_test src/level2_integration_test.cpp)
ament_target_dependencies(level2_integration_test rclcpp std_msgs )
target_link_libraries(level2_integration_test catch_ros2::catch_ros2_with_node_main  )


add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2 tf2_ros)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker ${cpp_typesupport_target})


add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

catch_ros2_add_integration_test (Level2_Integration_Test
  LAUNCH_FILE integration_test.launch.yaml  # example in YAML
  )

install(TARGETS
  level2_integration_test
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})


ament_package()
