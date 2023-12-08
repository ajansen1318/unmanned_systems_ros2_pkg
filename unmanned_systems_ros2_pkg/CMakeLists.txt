
cmake_minimum_required(VERSION 3.5)

#change name of package to this
project(unmanned_systems_ros2_pkg)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies this will allow CPP and python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
# Install Cpp executables
install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/python_node.py
  scripts/turtlebot_simple.py
  scripts/logger.py
  scripts/ros2_hw_4_node.py
  scripts/turtle_pid.py
  scripts/turtlebot_wp.py
  scripts/logger_node.py
  scripts/turtlebot_spawn.py
  scripts/pn_lab.py
  scripts/evader.py
  scripts/animated_pn.py
  aj_ws/evader_final.py
  log/plot.py
  aj_ws/hw5_p1_main.py
  aj_ws/hw5_p2_main.py
  aj_ws/hw5_p3_main.py  
  aj_ws/turtle/logger.py
  aj_ws/turtle/pid.py
  aj_ws/turtle/turtlebot_node.py
  aj_ws/turtle/utility.py 
  aj_ws/main.py
  aj_ws/turtle/animated_pn.py
  aj_ws/turtle/ProNav.py
  aj_ws/hw6_p1_main.py
  aj_ws/hw6_p1_main_old.py
  launch/run_hw6.launch.py
  aj_ws/pn.py
  aj_ws/pursuer_final.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

#launch
install(DIRECTORY
  launch
  # models
  # urdf
  # worlds
  # assets
  DESTINATION share/${PROJECT_NAME}
)