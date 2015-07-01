cmake_minimum_required(VERSION 2.8.3)
project(hrl_pr2_traj_playback)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs actionlib actionlib_msgs message_generation hrl_pr2_arms)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES TrajectoryPlay.action
)

add_message_files(
  DIRECTORY msg
  FILES TrajPlaybackCmd.msg
)

generate_messages(
    DEPENDENCIES std_msgs actionlib_msgs
) 

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS std_msgs actionlib actionlib_msgs message_runtime hrl_pr2_arms 
    INCLUDE_DIRS 
    LIBRARIES 
)

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DIRECTORY}
        USE_SOURSE_PERMISSIONS)

install(DIRECTORY test/
        DESTINATION ${CATKIN_PACKAGE_BIN_DIRECTORY}
        USE_SOURSE_PERMISSIONS)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
