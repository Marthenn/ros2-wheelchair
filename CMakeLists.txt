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
find_package(rosidl_default_generators REQUIRED)

# include cpp "include" directory
include_directories(include)

# generate ROS2 interfaces (cannot be run simultaneously with "ament_python_install_package")
rosidl_generate_interfaces(${PROJECT_NAME}
        "action/Move.action"
)

# install python modules (cannot be run simultaneously with "rosidl_generate_interfaces")
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
        wheelchair/speech.py
        wheelchair/alert.py
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()
