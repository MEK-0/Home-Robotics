cmake_minimum_required(VERSION 3.8)
project(home_robotics_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(joint_space_validation src/joint_space_validation.cpp)
add_executable(cartesian_pose_validation src/cartesian_pose_validation.cpp)
add_executable(planning_scene_environment src/planning_scene_environment.cpp)
ament_target_dependencies(planning_scene_environment
  ament_index_cpp
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  yaml_cpp_vendor
)
target_link_libraries(planning_scene_environment yaml-cpp)
ament_target_dependencies(cartesian_pose_validation
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
)

ament_target_dependencies(joint_space_validation
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
)

install(TARGETS joint_space_validation cartesian_pose_validation planning_scene_environment
  DESTINATION lib/${PROJECT_NAME}
)

# Install the single authoritative scene definition used by the MuJoCo builder.
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/scene.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py)
endif()

ament_package()
