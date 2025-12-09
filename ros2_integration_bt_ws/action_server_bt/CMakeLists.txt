cmake_minimum_required(VERSION 3.8)
project(action_server_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_DEPENDENCIES builtin_interfaces)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunTree.action"
  DEPENDENCIES ${MSG_DEPENDENCIES}
)

add_executable(bt_action_server
  src/bt_action_server_main.cpp
  src/bt_action_server_node.cpp
)

target_include_directories(bt_action_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_action_server
  rclcpp
  rclcpp_action
  ament_index_cpp
)

target_link_libraries(bt_action_server
  behaviortree_cpp::behaviortree_cpp
)

rosidl_target_interfaces(bt_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  bt_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()


