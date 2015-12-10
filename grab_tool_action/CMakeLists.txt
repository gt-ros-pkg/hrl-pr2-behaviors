cmake_minimum_required(VERSION 2.8.3)
project(grab_tool_action)

set(MSG_DEPS
    std_msgs
    geometry_msgs
    pr2_controllers_msgs
    pr2_common_action_msgs
    actionlib_msgs
    hrl_haptic_manipulation_in_clutter_srvs
    hrl_haptic_manipulation_in_clutter_msgs
    hrl_msgs
)

find_package(catkin REQUIRED COMPONENTS
        genmsg
        rospy
        tf
        tf2
        message_generation
        ${MSG_DEPS}
        actionlib
        
)

catkin_python_setup()

# generate messages
add_action_files(DIRECTORY action FILES GrabTool.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES ar_track_alvar
  DEPENDS
  CATKIN_DEPENDS
        std_msgs
        tf
        tf2
        message_runtime
        geometry_msgs
        ${MSG_DEPS}
        actionlib
       
)
catkin_install_python(PROGRAMS
    scripts/grab_tool_server
    scripts/grab_tool_client
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
