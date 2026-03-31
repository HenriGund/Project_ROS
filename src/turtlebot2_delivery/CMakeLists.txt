cmake_minimum_required(VERSION 3.0.2)
project(turtlebot2_delivery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  cv_bridge
  image_transport
)

catkin_package()

catkin_install_python(PROGRAMS
  src/mission_manager.py
  src/load_detector.py
  src/hmi_interface.py
  src/alert_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
