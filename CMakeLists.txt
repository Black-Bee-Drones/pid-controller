cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

# Add PID controller library
add_library(pid src/pid.cpp)
target_include_directories(pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pid PUBLIC cxx_std_17)

# Add PID controller node library (for component)
add_library(pid_controller_node SHARED src/pid_controller_node.cpp)
target_include_directories(pid_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pid_controller_node PUBLIC cxx_std_17)
target_link_libraries(pid_controller_node pid)
ament_target_dependencies(pid_controller_node
  rclcpp
  std_msgs
  rcl_interfaces
  rclcpp_components
)

# Register component
rclcpp_components_register_node(pid_controller_node
  PLUGIN "pid_controller::PIDControllerNode"
  EXECUTABLE pid_controller_node_exec
)

# Add standalone executable that uses the library
add_executable(pid_controller_standalone src/pid_controller_node_main.cpp)
target_link_libraries(pid_controller_standalone pid_controller_node)
target_include_directories(pid_controller_standalone PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add test environment executable
add_executable(pid_test_environment src/pid_test_environment.cpp)
target_include_directories(pid_test_environment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pid_test_environment
  rclcpp
  std_msgs
)

# Install libraries
install(TARGETS 
  pid
  pid_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS 
  pid_controller_standalone
  pid_controller_node_exec
  pid_test_environment
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files (when added)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
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
