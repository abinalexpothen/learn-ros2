cmake_minimum_required(VERSION 3.8)
project(learn_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(node_init_option1 src/node_init_option1.cpp)
ament_target_dependencies(node_init_option1 rclcpp)

add_executable(node_init_option2 src/node_init_option2.cpp)
ament_target_dependencies(node_init_option2 rclcpp)

add_library(node_init_components SHARED src/node_init_components.cpp)
# For Windows compatibility
target_compile_definitions(node_init_components PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(node_init_components rclcpp rclcpp_components)
rclcpp_components_register_nodes(node_init_components "our_namespace::OurClass")

install(TARGETS
node_init_option1
node_init_option2
node_init_components
DESTINATION lib/${PROJECT_NAME})


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
