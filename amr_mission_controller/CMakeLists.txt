cmake_minimum_required(VERSION 3.8)

project(amr_mission_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(nlohmann_json REQUIRED)



# ============================================================
# GraphManager
# ============================================================

add_library(
    graph_manager
    src/graph_manager.cpp
)

target_include_directories(
    graph_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    graph_manager
    nlohmann_json::nlohmann_json
)


# ============================================================
# StationManager
# ============================================================

add_library(
    station_manager
    src/station_manager.cpp
)

target_include_directories(
    station_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    station_manager
    nlohmann_json::nlohmann_json
)


# ============================================================
# NavigationManager
# ============================================================

add_library(
    navigation_manager
    src/navigation_manager.cpp
)

target_include_directories(
    navigation_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
    navigation_manager
    rclcpp
    rclcpp_action
    nav2_msgs
)

target_link_libraries(
    navigation_manager
    graph_manager
)

# ============================================================
# MissionManager
# ============================================================

add_library(
    mission_manager
    src/mission_manager.cpp
)

target_include_directories(
    mission_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
    mission_manager
    rclcpp
)

target_link_libraries(
    mission_manager
    navigation_manager
)
# ============================================================
# Main Node
# ============================================================

add_executable(
    graph_receiver_node
    src/graph_receiver_node.cpp
)

target_include_directories(
    graph_receiver_node PRIVATE
    include
)

ament_target_dependencies(
    graph_receiver_node
    rclcpp
    std_msgs
    tf2
    tf2_ros
    geometry_msgs
)

target_link_libraries(
    graph_receiver_node
    graph_manager
    station_manager
    navigation_manager
    mission_manager
)

# ============================================================
# Install
# ============================================================

install(
    TARGETS
        graph_manager
        station_manager
        navigation_manager
        mission_manager
        graph_receiver_node

    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)


ament_package()
