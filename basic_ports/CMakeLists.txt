cmake_minimum_required(VERSION 3.5)
project(basic_ports)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Package BehaviorTree.CPP v4 installé dans /usr/local/share/behaviortree_cpp
set(behaviortree_cpp_DIR "/usr/local/share/behaviortree_cpp/cmake")
find_package(behaviortree_cpp REQUIRED)
set(ament_index_cpp_DIR "/opt/ros/jazzy/share/ament_index_cpp/cmake")
find_package(ament_index_cpp REQUIRED)

add_executable(basic_ports_tree main.cpp)
target_include_directories(basic_ports_tree PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(basic_ports_tree
    behaviortree_cpp::behaviortree_cpp
    ament_index_cpp::ament_index_cpp
)
set_target_properties(basic_ports_tree PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
