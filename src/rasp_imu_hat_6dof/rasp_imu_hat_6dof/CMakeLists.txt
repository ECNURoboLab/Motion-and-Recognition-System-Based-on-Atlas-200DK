cmake_minimum_required(VERSION 2.8.3)
project(rasp_imu_hat_6dof)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    dynamic_reconfigure)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/imu.cfg
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS 
	nodes/imu_node.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
