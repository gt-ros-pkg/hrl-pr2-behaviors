cmake_minimum_required(VERSION 2.8.3)
project(hrl_pr2_arms)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy visualization_msgs pr2_controllers_msgs actionlib rosparam trajectory_msgs pr2_mechanism_msgs object_manipulation_msgs robot_mechanism_controllers pr2_manipulation_controllers pykdl_utils)

catkin_python_setup()

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS rospy visualization_msgs pr2_controllers_msgs actionlib rosparam trajectory_msgs pr2_mechanism_msgs object_manipulation_msgs robot_mechanism_controllers pr2_manipulation_controllers pykdl_utils
    INCLUDE_DIRS 
    LIBRARIES 
)

