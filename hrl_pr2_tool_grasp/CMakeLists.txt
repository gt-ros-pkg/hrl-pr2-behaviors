cmake_minimum_required(VERSION 2.8.3)
project(hrl_pr2_tool_grasp)
find_package(catkin REQUIRED COMPONENTS rospy ar_track_alvar actionlib actionlib_msgs message_generation geometry_msgs pr2_controllers_msgs)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES ARToolGrasp.action
)

generate_messages(
    DEPENDENCIES geometry_msgs actionlib_msgs
)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS rospy ar_track_alvar actionlib actionlib_msgs message_runtime geometry_msgs pr2_controllers_msgs
    INCLUDE_DIRS 
    LIBRARIES 
)

catkin_install_python(PROGRAMS scripts/ar_tool_grasp_server scripts/ar_tool_grasp_test_client
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
