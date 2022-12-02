cmake_minimum_required(VERSION 3.5)
project(rrrbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(publisher src/position_publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(switch src/controller_switch.cpp)
ament_target_dependencies(switch rclcpp controller_manager_msgs)

add_executable(pos_control src/position_control.cpp)
ament_target_dependencies(pos_control rclcpp std_msgs sensor_msgs tutorial_interfaces)

add_executable(theta_new_ref src/theta_ref_server.cpp)
ament_target_dependencies(theta_new_ref rclcpp std_msgs sensor_msgs tutorial_interfaces)

add_executable(theta_new_ref_client src/position_control.cpp)
ament_target_dependencies(theta_new_ref_client rclcpp std_msgs sensor_msgs tutorial_interfaces)

add_executable(main_control src/main_controller.cpp)
ament_target_dependencies(main_control rclcpp std_msgs sensor_msgs tutorial_interfaces)


install(TARGETS
  publisher
  switch
  pos_control
  theta_new_ref
  theta_new_ref_client
  main_control
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
