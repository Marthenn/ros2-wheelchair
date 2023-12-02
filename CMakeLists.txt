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
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# include cpp "include" directory
include_directories(include)

# install python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
        wheelchair/speech.py
        wheelchair/alert.py
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(odomSub src/OdomSub.cpp)
ament_target_dependencies(odomSub rclcpp nav_msgs)
install(TARGETS odomSub DESTINATION lib/${PROJECT_NAME})

ament_package()
