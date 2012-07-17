#!/usr/bin/env python

import roslib
roslib.load_manifest('hrl_pr2_ar_servo')
roslib.load_manifest('dynamic_reconfigure')
import rospy

from dynamic_reconfigure.server import Server
from hrl_pr2_ar_servo.cfg import PR2ARServoConfig

def callback(config, level):
    rospy.loginfo("""Reconfiugre Request: {black_cap}, {white_cap}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("pr2_servo_config", anonymous = True)

    srv = Server(PR2ARServoConfig, callback)
    rospy.spin()

