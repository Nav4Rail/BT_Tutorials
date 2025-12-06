cmake_minimum_required(VERSION 3.8)
project(ros2_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Obligatoire en C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)

# Ajout d'un exécutable pour l'exemple de Fibonacci manuel
add_executable(bt_node src/bt_node.cpp)

target_include_directories(bt_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Lier les dépendances à l'exécutable
ament_target_dependencies(bt_node
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  behaviortree_ros2
  ament_index_cpp
  action_tutorials_interfaces
)

# Installer l'exécutable
install(TARGETS
  bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installer les headers publics
install(
  DIRECTORY include/
  DESTINATION include
)

# Installer les arbres XML dans share/
install(DIRECTORY
  trees/
  DESTINATION share/${PROJECT_NAME}/trees
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
