cmake_minimum_required(VERSION 3.5)
project(rover_config)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

include_directories(include)

set(dependencies "ament_index_cpp" "rclcpp" "urdf")

add_executable(parser src/parser.cpp src/robot.cpp)
ament_target_dependencies(parser ${dependencies})

install(TARGETS
  parser
  DESTINATION lib/${PROJECT_NAME})

ament_package()
