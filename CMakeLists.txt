#  Copyright (c) 2022 Jonas Mahler

#  This file is part of pcl_example.

#  pcl_example is free software: you can redistribute it and/or modify it under the terms 
#  of the GNU General Public License as published by the Free Software Foundation, 
#  either version 3 of the License, or (at your option) any later version.

#  pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
#  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
#  See the GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License along 
#  with pcl_example. If not, see <https://www.gnu.org/licenses/>. 

cmake_minimum_required(VERSION 3.5)
project(pcl_example)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_example_node src/pcl_example_node.cpp src/pcl_example.cpp)

target_link_libraries( pcl_example_node
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(
  pcl_example_node
  rclcpp std_msgs 
  sensor_msgs
  pcl_conversions
)

install(TARGETS
  pcl_example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Files for testing
install(FILES
  test/cube1_record_1.yaml test/cube1_record_2.yaml test/pcl_example_launch.testing.py 
  DESTINATION lib/${PROJECT_NAME}
)

# Rviz2 config for visualization
install(FILES config/pcl_example.rviz 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake)    
  add_launch_test(test/pcl_example_launch.testing.py)
endif()

ament_package()
