cmake_minimum_required(VERSION 3.5)
project(drone_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories for header files
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# manual_control
add_executable(manual_control src/manual_control.cpp)
ament_target_dependencies(manual_control rclcpp px4_msgs)
install(TARGETS manual_control DESTINATION lib/${PROJECT_NAME})

# offboard_control
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# Export information to downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
