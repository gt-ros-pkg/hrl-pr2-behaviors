#! /usr/bin/env python
#
# Class for interacting with a PR2 JointTrajectory realtime controller.
#
# Copyright (c) 2012, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Kelsey Hawkins

import numpy as np
import copy

import rospy
import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint

from ep_arm_base import EPArmBase


class PR2ArmJointTraj(EPArmBase):
    '''Class for interacting with the JointTrajectory controller on the PR2.
        Controller type: robot_mechanism_controllers/JointTrajectoryActionController
        The equilibrium points are lists of joint angles.
    '''
    def __init__(self, arm_side, urdf, base_link='torso_lift_link', end_link='%s_gripper_tool_frame',
                 controller_name='/%s_arm_controller', kdl_tree=None, timeout=1.):
        super(PR2ArmJointTraj, self).__init__(arm_side, urdf, base_link, end_link,
                                              controller_name, kdl_tree, timeout)
        self.joint_action_client = actionlib.SimpleActionClient(
            self.controller_name + '/joint_trajectory_action',
            JointTrajectoryAction)

        self._state_sub = rospy.Subscriber(self.controller_name + '/state',
                                           JointTrajectoryControllerState, self._ctrl_state_cb)
        if self.wait_for_joint_angles(0):
            if not self.wait_for_ep(timeout):
                rospy.logwarn("[pr2_arm_joint_traj] Timed out waiting for EP.")
            elif not self.joint_action_client.wait_for_server(rospy.Duration(timeout)):
                rospy.logwarn("[pr2_arm_joint_traj] JointTrajectoryAction action server timed out.")

    def _ctrl_state_cb(self, ctrl_state):
        self._save_ep(np.array(ctrl_state.desired.positions))
        with self.ctrl_state_lock:
            self.ctrl_state_dict["frame"] = ctrl_state.header.frame_id
            self.ctrl_state_dict["x_desi"] = np.array(ctrl_state.desired.positions)
            self.ctrl_state_dict["xd_desi"] = np.array(ctrl_state.desired.velocities)
            self.ctrl_state_dict["xdd_desi"] = np.array(ctrl_state.desired.accelerations)
            self.ctrl_state_dict["x_act"] = np.array(ctrl_state.actual.positions)
            self.ctrl_state_dict["xd_act"] = np.array(ctrl_state.actual.velocities)
            self.ctrl_state_dict["xdd_act"] = np.array(ctrl_state.actual.accelerations)
            self.ctrl_state_dict["x_err"] = np.array(ctrl_state.error.positions)
            self.ctrl_state_dict["xd_err"] = np.array(ctrl_state.error.velocities)
            self.ctrl_state_dict["xdd_err"] = np.array(ctrl_state.error.accelerations)

    ##
    # Returns the current equilibrium point
    # @return equilibrium point
    def get_ep(self, wrapped=False):
        with self.ep_lock:
            ret_ep = copy.copy(self.ep)
        if wrapped:
            return self.wrap_angles(ret_ep)
        else:
            return ret_ep

    ##
    # Commands joint angles to a single position
    # @param jep List of joint params to command the the arm to reach
    # @param duration Length of time command should take
    # @param delay Time to wait before starting joint command
    def set_ep(self, jep, duration, delay=0.0):
        jtg = JointTrajectoryGoal()
        jtg.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(delay)
        jtg.trajectory.joint_names = self.get_joint_names()
        jtp = JointTrajectoryPoint()
        jtp.positions = list(jep)
        jtp.time_from_start = rospy.Duration(duration)
        jtg.trajectory.points.append(jtp)
        self.joint_action_client.send_goal(jtg)

    ##
    # Interpolates from one ep to the other using the trajectory specified by t.
    # @param ep_a The starting EP.
    # @param ep_b The ending EP.
    # @param t_vals List of values in [0, 1] representing where in the linear interpolation
    #               of ep_a to ep_b to place that point.
    # @return np.array of size len(t_vals)xlen(ep_*) trajectory.
    def interpolate_ep(self, ep_a, ep_b, t_vals):
        linspace_list = [[ep_a[i] + (ep_b[i] - ep_a[i]) * t for t in t_vals]
                         for i in range(len(ep_a))]
        return np.dstack(linspace_list)[0]
