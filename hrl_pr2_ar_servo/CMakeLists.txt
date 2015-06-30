cmake_minimum_required(VERSION 2.8.3)
project(hrl_pr2_ar_servo)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS image_geometry tf_conversions pykdl_utils ar_track_alvar)
find_package(OpenCV REQUIRED)

#uncomment if you have defined messages
add_message_files(
  DIRECTORY msg
  FILES ARServoGoalData.msg
)
#uncomment if you have defined services

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES geometry_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS pcl pcl_ros image_geometry tf_conversions pykdl_utils ar_pose
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(ar_confirm src/ar_confirm.cpp)
add_executable(ar_servo_image_proc src/ar_servo_image_proc.cpp)

target_link_libraries(ar_confirm ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ar_servo_image_proc ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

install(TARGETS ar_confirm ar_servo_image_proc
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
