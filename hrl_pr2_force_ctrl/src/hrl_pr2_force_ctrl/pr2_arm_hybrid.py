#! /usr/bin/python
#
# Class for interacting with the hybrid force realtime controller.
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

import roslib
roslib.load_manifest('hrl_pr2_force_ctrl')

import rospy
from pr2_manipulation_controllers.msg import JTTaskControllerState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray, Header, Bool

from ep_arm_base import EPArmBase, create_ep_arm
from pr2_arm_cart_base import PR2ArmCartPostureBase, extract_twist
from hrl_geom.pose_converter import PoseConv

from hrl_pr2_arms.pr2_arm import PR2ArmCartesianPostureBase, extract_twist
from msg import HybridCartesianGains, HybridForceState

##
# Class for interacting with the Cartesian controllers on the PR2.
# Controller type: hrl_pr2_force_ctrl/HybridForce
# The equilibrium points are pose-like objects.
class PR2ArmHybridForce(PR2ArmCartPostureBase):
    def __init__(self, arm_side, urdf, base_link='torso_lift_link', end_link='%s_gripper_tool_frame', 
                 controller_name='/%s_cart', kdl_tree=None, timeout=1.):
        super(PR2ArmJTranspose, self).__init__(arm_side, urdf, base_link, end_link, 
                                               controller_name, kdl_tree, timeout)
        self.auto_update = False
        self.command_gains_pub = rospy.Publisher(controller_name + '/gains', HybridCartesianGains)
        self.command_force_pub = rospy.Publisher(controller_name + '/command_force', WrenchStamped)
        self.command_force_max_pub = rospy.Publisher(controller_name + '/command_max_force', 
                                                     WrenchStamped)
        self.command_zero_pub = rospy.Publisher(controller_name + '/ft_zero', Bool)
        self.ft_wrench_sub = rospy.Subscriber(controller_name + '/ft_wrench', WrenchStamped, 
                                              self._ft_wrench_cb)
        self.ft_wrench = np.mat(6 * [0]).T
        self.tip_frame = rospy.get_param(controller_name + '/tip_name')
        self.force_selector = 6 * [0]

        self.trans_p_motion_gains = 3 * [rospy.get_param(controller_name + '/cart_gains/trans/p')]
        self.trans_d_motion_gains = 3 * [rospy.get_param(controller_name + '/cart_gains/trans/d')]

        self.rot_p_motion_gains = 3 * [rospy.get_param(controller_name + '/cart_gains/rot/p')]
        self.rot_d_motion_gains = 3 * [rospy.get_param(controller_name + '/cart_gains/rot/d')]

        self.trans_p_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/trans/p')]
        self.trans_i_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/trans/i')]
        self.trans_i_max_force_gains = 3 * [rospy.get_param(controller_name + 
                                                            '/force_gains/trans/i_max')]

        self.rot_p_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/rot/p')]
        self.rot_i_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/rot/i')]
        self.rot_i_max_force_gains = 3 * [rospy.get_param(controller_name + '/force_gains/rot/i_max')]

        self.ctrl_state_dict = {}
        rospy.Subscriber(self.controller_name + '/state', JTTaskControllerState, 
                         self._ctrl_state_cb)
        if self.wait_for_joint_angles(0):
            if not self.wait_for_ep(timeout):
                rospy.logwarn("[pr2_arm_hybrid] Timed out waiting for EP.")

    def _ctrl_state_cb(self, ctrl_state):
        with self.ep_lock:
            self.ep = PoseConv.to_homo_mat(ctrl_state.x_desi_filtered)
        with self.ctrl_state_lock:
            self.ctrl_state_dict["frame"] = ctrl_state.header.frame_id
            self.ctrl_state_dict["x_desi"] = PoseConverter.to_pos_rot(ctrl_state.x_desi)
            self.ctrl_state_dict["xd_desi"] = extract_twist(ctrl_state.xd_desi)
            self.ctrl_state_dict["x_act"] = PoseConverter.to_pos_rot(ctrl_state.x)
            self.ctrl_state_dict["xd_act"] = extract_twist(ctrl_state.xd)
            self.ctrl_state_dict["x_desi_filt"] = PoseConverter.to_pos_rot(
                                                                ctrl_state.x_desi_filtered)
            self.ctrl_state_dict["x_err"] = extract_twist(ctrl_state.x_err)
            self.ctrl_state_dict["tau_pose"] = np.array(ctrl_state.tau_pose)
            self.ctrl_state_dict["tau_posture"] = np.array(ctrl_state.tau_posture)
            self.ctrl_state_dict["tau"] = np.array(ctrl_state.tau)
            self.ctrl_state_dict["F"] = np.array([ctrl_state.F.force.x, 
                                                  ctrl_state.F.force.y,
                                                  ctrl_state.F.force.z,
                                                  ctrl_state.F.torque.x,
                                                  ctrl_state.F.torque.y,
                                                  ctrl_state.F.torque.z])

    def _ft_wrench_cb(self, ws):
        self.ft_wrench = np.mat([ws.wrench.force.x, ws.wrench.force.y, ws.wrench.force.z, 
                                 ws.wrench.torque.x, ws.wrench.torque.y, ws.wrench.torque.z]).T

    def get_ft_wrench(self):
        return self.ft_wrench

    def set_motion_gains(self, p_trans=None, p_rot=None, d_trans=None, d_rot=None):
        local_names = ['trans_p_motion_gains', 'rot_p_motion_gains', 
                       'trans_d_motion_gains', 'rot_d_motion_gains']
        vals = [p_trans, p_rot, d_trans, d_rot]
        for local_name, val in zip(local_names, vals):
            self._set_gain(local_name, val)
        if self.auto_update:
            self.update_gains()

    def set_force_gains(self, p_trans=None, p_rot=None, i_trans=None, i_rot=None, 
                              i_max_trans=None, i_max_rot=None):
        local_names = ['trans_p_force_gains', 'rot_p_force_gains',
                       'trans_i_force_gains','rot_i_force_gains',
                       'trans_i_max_force_gains','rot_i_max_force_gains']
        vals = [p_trans, p_rot, i_trans, i_rot, i_max_trans, i_max_rot]
        for local_name, val in zip(local_names, vals):
            self._set_gain(local_name, val)
        if self.auto_update:
            self.update_gains()

    def _set_gain(self, local_name, val):
        if val is not None:
            try:
                if len(val) == 3:
                    self.__dict__[local_name] = list(val)
                else:
                    raise Exception()
            except Exception as e:
                if type(val) in [int, float]:
                    self.__dict__[local_name] = 3 * [float(val)]
                else:
                    rospy.logerr("Bad gain parameter (must be single value or array-like of 3)")

    def set_tip_frame(self, tip_frame):
        self.tip_frame = tip_frame
        if self.auto_update:
            self.update_gains()

    def set_force_directions(self, directions):
        if len(directions) == 6 and all([direction in [0, 1] for direction in directions]):
            self.force_selector = list(directions)
            return
        self.force_selector = 6 * [0]
        names = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        for direction in directions:
            if direction in names:
                self.force_selector[names.index(direction)] = 1
        if self.auto_update:
            self.update_gains()

    def set_mass_params(self, mass, center_of_mass=None):
        params_msg = Float64MultiArray()
        params_msg.data = [mass]
        if center_of_mass is not None:
            params_msg.data += list(center_of_mass)

    def update_gains(self):
        p_gains = self.trans_p_motion_gains + self.rot_p_motion_gains
        d_gains = self.trans_d_motion_gains + self.rot_d_motion_gains
        fp_gains = self.trans_p_force_gains + self.rot_p_force_gains
        fi_gains = self.trans_i_force_gains + self.rot_i_force_gains
        fi_max_gains = self.trans_i_max_force_gains + self.rot_i_max_force_gains
        gains_msg = HybridCartesianGains(Header(0, rospy.Time.now(), self.tip_frame),
                                         p_gains, d_gains, 
                                         fp_gains, fi_gains, fi_max_gains, self.force_selector)
        self.command_gains_pub.publish(gains_msg)

    def set_force(self, f, frame=None):
        if frame is None:
            frame = self.tip_frame
        f_msg = WrenchStamped()
        f_msg.header.stamp = rospy.Time.now()
        f_msg.header.frame_id = frame
        f_msg.wrench.force.x, f_msg.wrench.force.y, f_msg.wrench.force.z = f[0], f[1], f[2]
        f_msg.wrench.torque.x, f_msg.wrench.torque.y, f_msg.wrench.torque.z = f[3], f[4], f[5]
        self.command_force_pub.publish(f_msg)

    def set_force_max(self, f):
        f_msg = WrenchStamped()
        f_msg.header.stamp = rospy.Time.now()
        f_msg.wrench.force.x, f_msg.wrench.force.y, f_msg.wrench.force.z = f[0], f[1], f[2]
        f_msg.wrench.torque.x, f_msg.wrench.torque.y, f_msg.wrench.torque.z = f[3], f[4], f[5]
        self.command_force_max_pub.publish(f_msg)

    def zero_sensor(self):
        bool_msg = Bool()
        self.command_zero_pub.publish(bool_msg)

    def use_auto_update(self, use_auto_update):
        self.auto_update = use_auto_update
