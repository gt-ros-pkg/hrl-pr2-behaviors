cmake_minimum_required(VERSION 2.8.3)
project(hrl_pr2_arms)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy visualization_msgs pr2_controllers_msgs actionlib rosparam trajectory_msgs pr2_mechanism_msgs manipulation_msgs robot_mechanism_controllers pykdl_utils)

catkin_python_setup()

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS rospy visualization_msgs pr2_controllers_msgs actionlib rosparam trajectory_msgs pr2_mechanism_msgs manipulation_msgs robot_mechanism_controllers pykdl_utils
    INCLUDE_DIRS 
    LIBRARIES 
)

catkin_install_python(PROGRAMS scripts/carefree_switch_controller
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
