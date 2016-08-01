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
        head_kin = create_kdl_kin('torso_lift_link', 'head_tilt_link', description_param="/robot_description")
        self.limits = {'right_arm': r_arm_kin.get_joint_limits(),
                       'left_arm':  l_arm_kin.get_joint_limits(),
                       'head':  head_kin.get_joint_limits()}

        self.motors_halted = None
        self.outside_limits = None
        self.reset_timer = None

        self.emulate_runstop_service = rospy.ServiceProxy('/emulate_runstop', SetRunStop, persistent=True)
        self.motor_status_sub = rospy.Subscriber('/pr2_ethercat/motors_halted', Bool, self.motor_state_cb)
        self.r_arm_state_sub = rospy.Subscriber('/r_arm_controller/state', JointTrajectoryControllerState, self.joint_state_cb, 'right_arm')
        self.l_arm_state_sub = rospy.Subscriber('/l_arm_controller/state', JointTrajectoryControllerState, self.joint_state_cb, 'left_arm')
        self.head_state_sub = rospy.Subscriber('/head_traj_controller/state', JointTrajectoryControllerState, self.joint_state_cb, 'head')
        rospy.loginfo("[%s] Ready", rospy.get_name())

    def joint_state_cb(self, msg, limb):
        if self.motors_halted:
            return
        joints = np.array(msg.actual.positions)
        limits = self.limits[limb]
        if any(joints < limits[0]) or any(joints > limits[1]):
            rospy.logwarn("[%s] Joints in %s outside soft limits.", rospy.get_name(), limb)
#            print limits[1]
#            print joints
#            print limits[0]
            self.outside_limits = True
            if self.reset_timer is None:  # Expect joints to violate soft limits when motors are halted
                self.halt_motors()
        else:
            self.outside_limits = False

    def motor_state_cb(self, state_msg):
        if state_msg.data:
            if self.reset_timer is not None:  # Clear any running reset timers, since we're stopped again
                self.reset_timer.shutdown()
                self.reset_time = None
        if not state_msg.data and self.motors_halted:  # Now not halted, but were, so we've reset.  Create timer to check limits after reset
            self.reset_timer = rospy.Timer(rospy.Duration(4), self.check_reset, oneshot=True)
        self.motors_halted = state_msg.data  # Always update to latest state

    def check_reset(self, event):
        if self.outside_limits:
            rospy.logwarn("[%s] Joints outside soft limits after reset.", rospy.get_name())
            self.halt_motors()
        else:
            rospy.loginfo("[%s] Joints within limits after reset.", rospy.get_name())
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
