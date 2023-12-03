cmake_minimum_required(VERSION 3.8)
project(wheelchair)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wheelchair_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# include cpp "include" directory
include_directories(include)

# action server
add_library(action_server SHARED
  src/Movement.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "WHEELCHAIR_BUILDING_DLL")
ament_target_dependencies(action_server
  "wheelchair_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_geometry_msgs"
)
rclcpp_components_register_node(action_server PLUGIN "wheelchair::MovementActionServer" EXECUTABLE movement_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# action client
add_executable(movement_action_client src/Handler.cpp)
ament_target_dependencies(movement_action_client
  "wheelchair_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
)
install(TARGETS
  movement_action_client
  DESTINATION lib/${PROJECT_NAME})

# install python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
        wheelchair/speech.py
        wheelchair/alert.py
        DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)


ament_package()
