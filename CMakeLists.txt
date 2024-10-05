cmake_minimum_required(VERSION 3.8)
project(trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	system 
	thread 
	program_options
)
find_package(rosidl_default_generators REQUIRED)  # To geneate custom message

find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
include_directories(include 
  /opt/ros/humble/include/ompl-1.6
  #/usr/include/3dPathPlanner
)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# add_library(3dPathPlanner SHARED src/lib/planner.cpp)
# ament_target_dependencies(3dPathPlanner ompl fcl Eigen3)
# target_include_directories(3dPathPlanner PUBLIC
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
# 	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
# )


# /offboard_control
add_executable(offboard_control src/offboard/offboard_control.cpp src/offboard/Trajectory.cpp src/offboard/spline/planner_spline.cpp src/offboard/planner/planner.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs geometry_msgs Eigen3 Boost tf2 ompl fcl octomap octomap_msgs nav_msgs)
target_include_directories(offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(offboard_control Eigen3::Eigen ${geometry_msgs_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} fcl) # ${ompl_LIBRARIES} fcl)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})


add_executable(move_manager_node src/move_manager_node.cpp)
ament_target_dependencies(move_manager_node rclcpp std_msgs geometry_msgs px4_msgs nav_msgs Boost tf2 tf2_ros)
install(TARGETS move_manager_node DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs nav_msgs fcl ompl)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)


# ament_export_targets(export_3dPathPlanner HAS_LIBRARY_TARGET)
# ament_export_libraries(3dPathPlanner)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms 
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# install(TARGETS 3dPathPlanner
# 	EXPORT export_3dPathPlanner 
# 	ARCHIVE DESTINATION lib
# 	LIBRARY DESTINATION lib
# 	RUNTIME DESTINATION bin
# 	INCLUDES DESTINATION include
# )

# Install launch files.
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MoveCmd.msg"
  DEPENDENCIES geometry_msgs std_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(offboard_control "${cpp_typesupport_target}")
target_link_libraries(move_manager_node "${cpp_typesupport_target}")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
