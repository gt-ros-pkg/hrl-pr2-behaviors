#!/usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import Bool

from pr2_controllers_msgs.msg import JointTrajectoryControllerState

from pykdl_utils.kdl_kinematics import create_kdl_kin

from hrl_pr2_upstart.srv import SetRunStop, SetRunStopRequest


class JointLimitWatchdog(object):
    def __init__(self):
        r_arm_kin = create_kdl_kin('torso_lift_link', 'r_gripper_tool_frame', description_param="/robot_description")
        l_arm_kin = create_kdl_kin('torso_lift_link', 'l_gripper_tool_frame', description_param="/robot_description")
        self.arm_limits = {'right': r_arm_kin.get_joint_limits(),
                           'left':  l_arm_kin.get_joint_limits()}

        self.motors_halted = None
        self.outside_limits = None
        self.reset_timer = None

        self.emulate_runstop_service = rospy.ServiceProxy('emulate_runstop', SetRunStop, persistent=True)
        self.r_arm_state_sub = rospy.Subscriber('r_arm_controller/state', JointTrajectoryControllerState, self.arm_state_cb, 'right')
        self.l_arm_state_sub = rospy.Subscriber('r_arm_controller/state', JointTrajectoryControllerState, self.arm_state_cb, 'left')
        self.motor_status_sub = rospy.Subscriber('pr2_ethercat/motors_halted', Bool, self.motor_state_cb)

    def arm_state_cb(self, msg, arm):
        if self.motors_halted or self.reset_timer is not None:
            # Expect joints to violate soft limits when motors are halted
            return
        joints = np.array(msg.actual.positions)
        limits = self.arm_limits[arm]
        if any(joints < limits[0]) or any(joints > limits[1]):
            self.halt_motors()
            rospy.logwarn("[%s] Joints outside soft limits.", rospy.get_name())
            self.outside_limits = True
        else:
            self.outside_limits = False

    def motor_state_cb(self, state_msg):
        if state_msg.data:
            if self.reset_timer is not None:
                self.reset_timer.shutdown()
                self.reset_time = None
            if not self.motors_halted:
                self.motors_halted = True
        elif self.motors_halted:  # Now not halted, but were
            self.reset_timer = rospy.Timer(rospy.Duration(4), self.check_reset, oneshot=True)

    def check_reset(self):
        if self.outside_limits:
            rospy.logwarn("[%s] Joints outside soft limits after reset.", rospy.get_name())
            self.halt_motors()
        self.reset_timer = None

    def halt_motors(self):
        req = SetRunStopRequest()
        req.stop = True
        self.emulate_runstop_service.call(req)
        rospy.logwarn("[%s] Halting Motors!", rospy.get_name())


def main():
    rospy.init_node('joint_limit_watchdog')
    watchdog = JointLimitWatchdog()
    rospy.spin()
