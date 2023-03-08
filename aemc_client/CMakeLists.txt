cmake_minimum_required(VERSION 3.8)
project(aemc_client)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_PREFIX_PATH /home/itr-wh/Work/Ros2_ws/install)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(aemc_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)
link_directories(lib)

add_executable(aemc_client_node src/aemc_client_node.cpp src/Aemc.cpp)
add_executable(test_node src/test.cpp src/Aemc.cpp)
add_executable(visualization_node src/visualization.cpp src/Aemc.cpp)
add_executable(sub_node src/sub.cpp)
ament_target_dependencies(aemc_client_node rclcpp std_msgs geometry_msgs aemc_msgs)
target_link_libraries(aemc_client_node seeker_sdk pthread)
ament_target_dependencies(visualization_node rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs aemc_msgs tf2 tf2_ros)
target_link_libraries(visualization_node seeker_sdk pthread)
ament_target_dependencies(sub_node rclcpp aemc_msgs)
ament_target_dependencies(test_node rclcpp std_msgs geometry_msgs visualization_msgs nav_msgs aemc_msgs tf2 tf2_ros)

install(TARGETS
        aemc_client_node
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS
        visualization_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
