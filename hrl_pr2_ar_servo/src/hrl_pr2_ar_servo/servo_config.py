#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from hrl_pr2_ar_servo.cfg import PR2ARServoConfig

NODE_NAME = 'ar_servo_image_proc'
PARAMS = ['black_cap', 'white_cap']


def callback(config, level):
    for param in PARAMS:
        rospy.set_param(NODE_NAME+'/'+param, config[param])
    return config

def main():
    rospy.init_node("ar_servo_config")
    srv = Server(PR2ARServoConfig, callback)
    rospy.spin()
