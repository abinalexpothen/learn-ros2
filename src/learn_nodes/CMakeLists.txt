cmake_minimum_required(VERSION 3.8)
project(learn_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"srv/LearnNodesExerciseService.srv"
)

add_executable(node_init_option1 src/node_init_option1.cpp)
ament_target_dependencies(node_init_option1 rclcpp)

add_executable(node_init_option2 src/node_init_option2.cpp)
ament_target_dependencies(node_init_option2 rclcpp)

add_executable(my_managed_node src/my_managed_node.cpp)
ament_target_dependencies(my_managed_node rclcpp rclcpp_lifecycle)

# component node: node_init_components
add_library(node_init_components SHARED src/node_init_components.cpp)
# Windows support
target_compile_definitions(node_init_components PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(node_init_components rclcpp rclcpp_components)
rclcpp_components_register_nodes(node_init_components "our_namespace::OurClass")

# component node: learn_nodes_node
add_library(learn_nodes_node SHARED src/learn_nodes_node.cpp)
# 1. Standard dependencies
ament_target_dependencies(learn_nodes_node rclcpp rclcpp_components lifecycle_msgs)
# 2. link it to the generated service ("typesupport targets")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(learn_nodes_node "${cpp_typesupport_target}")
# 3. Explicityly force headers to be generated BEFORE this library compiles
add_dependencies(learn_nodes_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
# Windows support
target_compile_definitions(learn_nodes_node PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(learn_nodes_node "learn_nodes_exercise::LearnNodesNode")

# component node: learn_nodes_managed_1
add_library(learn_nodes_managed_1 SHARED src/learn_nodes_managed_1.cpp)
# windows support
target_compile_definitions(learn_nodes_managed_1 PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(learn_nodes_managed_1 rclcpp rclcpp_lifecycle rclcpp_components)
rclcpp_components_register_nodes(learn_nodes_managed_1 "learn_nodes_exercise::LearnNodesManaged1")

# component node: learn_nodes_managed_2
add_library(learn_nodes_managed_2 SHARED src/learn_nodes_managed_2.cpp)
# windows support
target_compile_definitions(learn_nodes_managed_2 PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(learn_nodes_managed_2 rclcpp rclcpp_lifecycle rclcpp_components)
rclcpp_components_register_nodes(learn_nodes_managed_2 "learn_nodes_exercise::LearnNodesManaged2")

# Separate the libraries from the executables
install(TARGETS
  node_init_components
  learn_nodes_node
  learn_nodes_managed_1
  learn_nodes_managed_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  node_init_option1
  node_init_option2
  my_managed_node
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

# Use the simpler export command
ament_export_libraries(learn_nodes_node learn_nodes_managed_1 learn_nodes_managed_2)

# This is the vital line that creates the 'phone book' for ROS 2
ament_package()
