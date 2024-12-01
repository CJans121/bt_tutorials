cmake_minimum_required(VERSION 3.8)
project(bt_tutorials_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Include directories
include_directories(
  include
)

# Define common ROS dependencies
set(ROS_COMMON_DEPENDENCIES rclcpp behaviortree_cpp)

# Define libraries
set(PROJECT_LIBRARIES your_first_behavior_tree blackboard_and_ports ports_with_generic_types reactive_behaviors using_subtrees pass_additional_arguments)

# Create libraries and link against ROS dependencies
set(PROJECT_NODES)
foreach(LIBRARY ${PROJECT_LIBRARIES})
  # Store node names
  list(APPEND PROJECT_NODES ${LIBRARY}_node)

  # Create libraries
  add_library(${LIBRARY} src/${PROJECT_NAME}/${LIBRARY}.cpp)

  # Link against ROS dependencies
  ament_target_dependencies(${LIBRARY} ${ROS_COMMON_DEPENDENCIES})
endforeach()

# Manually add nodes not matching above pattern
list (APPEND PROJECT_NODES port_remapping_node use_multiple_xml_files_node scripting_example_node loggers_and_observer_node)

# Create node executables and link libraries against them
foreach(NODE ${PROJECT_NODES})
  # Create node executable
  add_executable(${NODE} src/${NODE}.cpp)

  # Link against ROS dependencies
  ament_target_dependencies(${NODE} ${ROS_COMMON_DEPENDENCIES})

  # Link against non-ROS dependencies
  target_link_libraries(${NODE} ${PROJECT_LIBRARIES})
endforeach()

# Install include directories
install(
  DIRECTORY include
  DESTINATION include
)

# Install libraries
install(
  TARGETS ${PROJECT_LIBRARIES}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(
  TARGETS ${PROJECT_NODES}
  DESTINATION lib/${PROJECT_NAME}
)

# Build testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
