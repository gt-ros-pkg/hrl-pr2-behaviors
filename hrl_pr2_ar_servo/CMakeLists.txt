cmake_minimum_required(VERSION 2.8.3)
project(hrl_pr2_ar_servo)
find_package(catkin REQUIRED COMPONENTS image_geometry image_transport cv_bridge dynamic_reconfigure tf_conversions pykdl_utils ar_track_alvar_msgs message_generation geometry_msgs std_msgs)
find_package(OpenCV REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/PR2ARServo.cfg
)

#uncomment if you have defined messages
add_message_files(
  DIRECTORY msg
  FILES ARServoGoalData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS image_geometry image_transport cv_bridge dynamic_reconfigure tf_conversions pykdl_utils ar_track_alvar_msgs message_runtime geometry_msgs std_msgs
    INCLUDE_DIRS 
    LIBRARIES 
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(ar_confirm src/ar_confirm.cpp)
add_executable(ar_servo_image_proc src/ar_servo_image_proc.cpp)

target_link_libraries(ar_confirm ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(ar_servo_image_proc ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_install_python(PROGRAMS 
    scripts/ar_pose_to_alvar_relay
    scripts/servo_config
    scripts/pr2_ar_servo
    scripts/sm_pr2_servoing
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ar_confirm ar_servo_image_proc
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
