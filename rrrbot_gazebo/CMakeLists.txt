cmake_minimum_required(VERSION 3.5)
project(rrrbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
 include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_executable(publisher src/position_publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(switch src/controller_switch.cpp)
ament_target_dependencies(switch rclcpp controller_manager_msgs)

add_executable(pos_control src/position_control.cpp)
ament_target_dependencies(pos_control rclcpp std_msgs sensor_msgs tutorial_interfaces)

add_executable(main_control src/main_controller.cpp)
ament_target_dependencies(main_control rclcpp std_msgs sensor_msgs tutorial_interfaces)

add_executable(vel_kinematics src/velocity_kinematics.cpp)
ament_target_dependencies(vel_kinematics rclcpp std_msgs sensor_msgs tutorial_interfaces)

add_executable(velocity_control src/velocity_controller.cpp)
ament_target_dependencies(velocity_control rclcpp std_msgs sensor_msgs tutorial_interfaces)


install(TARGETS
  publisher
  switch
  pos_control
  main_control
  vel_kinematics
  velocity_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
