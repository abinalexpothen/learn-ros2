cmake_minimum_required(VERSION 3.8)
project(learn_executors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tracy_vendor REQUIRED)

add_executable(demo_server_node src/demo_server.cpp)
target_include_directories(demo_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(demo_server_node 
  rclcpp 
  std_srvs
  tracy_vendor
)

target_compile_definitions(demo_server_node PUBLIC TRACY_ENABLE)
target_link_libraries(demo_server_node tracy_vendor::TracyClient)

add_executable(deadlock_client_node src/deadlock_demo_client.cpp)
target_include_directories(deadlock_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(deadlock_client_node 
  rclcpp 
  std_srvs
  tracy_vendor
)

target_compile_definitions(deadlock_client_node PUBLIC TRACY_ENABLE)
target_link_libraries(deadlock_client_node tracy_vendor::TracyClient)

add_executable(fix_deadlock_client_node src/fix_deadlock_demo_client.cpp)
target_include_directories(fix_deadlock_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fix_deadlock_client_node
  rclcpp
  std_srvs
  tracy_vendor
)

target_compile_definitions(fix_deadlock_client_node PUBLIC TRACY_ENABLE)
target_link_libraries(fix_deadlock_client_node tracy_vendor::TracyClient)

add_executable(fix_deadlock_single_threaded_exec_node src/fix_deadlock_single_threaded_exec.cpp)
target_include_directories(fix_deadlock_single_threaded_exec_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fix_deadlock_single_threaded_exec_node
  rclcpp
  std_srvs
  tracy_vendor
)

target_compile_definitions(fix_deadlock_single_threaded_exec_node PUBLIC TRACY_ENABLE)
target_link_libraries(fix_deadlock_single_threaded_exec_node tracy_vendor::TracyClient)

install(TARGETS
  demo_server_node
  deadlock_client_node
  fix_deadlock_client_node
  fix_deadlock_single_threaded_exec_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
