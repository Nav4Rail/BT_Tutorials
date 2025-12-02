cmake_minimum_required(VERSION 3.5)
project(first_behavior_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Package BehaviorTree.CPP v4 installé dans /usr/local/share/behaviortree_cpp
set(behaviortree_cpp_DIR "/usr/local/share/behaviortree_cpp/cmake")
find_package(behaviortree_cpp REQUIRED)
set(ament_index_cpp_DIR "/opt/ros/jazzy/share/ament_index_cpp/cmake")
find_package(ament_index_cpp REQUIRED)

add_executable(first_tree main.cpp)
target_include_directories(first_tree PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(first_tree
    behaviortree_cpp::behaviortree_cpp
    ament_index_cpp::ament_index_cpp
)
set_target_properties(first_tree PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
