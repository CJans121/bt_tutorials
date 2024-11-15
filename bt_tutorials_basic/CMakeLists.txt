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

# Create libraries
add_library(your_first_behavior_tree src/${PROJECT_NAME}/your_first_behavior_tree.cpp)

# Store libraries in a variable for future use
set(PROJECT_LIBRARIES your_first_behavior_tree)

# Link all libraries against ROS dependencies
foreach(LIBRARY ${PROJECT_LIBRARIES})
  ament_target_dependencies(${LIBRARY} ${ROS_COMMON_DEPENDENCIES})
endforeach()

# Create ,store, and link node executables
set(PROJECT_NODES)
foreach(LIBRARY ${PROJECT_LIBRARIES})
  set(NODE ${LIBRARY}_node)
  add_executable(${NODE} src/${NODE}.cpp)
  list(APPEND PROJECT_NODES ${NODE})
  ament_target_dependencies(${NODE} ${ROS_COMMON_DEPENDENCIES})
  target_link_libraries(${NODE} ${LIBRARY})
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
