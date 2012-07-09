#! /usr/bin/python
#
# Class for interacting with the PR2 J Transpose realtime controller.
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
roslib.load_manifest('hrl_pr2_arms')

import rospy
from robot_mechanism_controllers.msg import JTCartesianControllerState

from ep_arm_base import EPArmBase, create_ep_arm
from pr2_arm_cart_base import PR2ArmCartPostureBase, extract_twist
from hrl_geom.pose_converter import PoseConv
import hrl_geom.transformations as trans

##
# Class for interacting with the Cartesian controllers on the PR2.
# Controller type: robot_mechanism_controllers/JTCartesianController
# The equilibrium points are pose-like objects.
class PR2ArmJTranspose(PR2ArmCartPostureBase):
    def __init__(self, arm_side, urdf, base_link='torso_lift_link', end_link='%s_gripper_tool_frame', 
                 controller_name='/%s_cart', kdl_tree=None, timeout=1.):
        super(PR2ArmJTranspose, self).__init__(arm_side, urdf, base_link, end_link, 
                                               controller_name, kdl_tree, timeout)
        rospy.Subscriber(self.controller_name + '/state', JTCartesianControllerState, 
                         self._ctrl_state_cb)
        if self.wait_for_joint_angles(0):
            if not self.wait_for_ep(timeout):
                rospy.logwarn("[pr2_arm_jt] Timed out waiting for EP.")

    def _ctrl_state_cb(self, ctrl_state):
        self._save_ep(PoseConv.to_homo_mat(ctrl_state.x_desi_filtered))
        with self.ctrl_state_lock:
            self.ctrl_state_dict["frame"] = ctrl_state.header.frame_id
            self.ctrl_state_dict["x_desi"] = PoseConv.to_pos_rot(ctrl_state.x_desi)
            self.ctrl_state_dict["xd_desi"] = extract_twist(ctrl_state.xd_desi)
            self.ctrl_state_dict["x_act"] = PoseConv.to_pos_rot(ctrl_state.x)
            self.ctrl_state_dict["xd_act"] = extract_twist(ctrl_state.xd)
            self.ctrl_state_dict["x_desi_filt"] = PoseConv.to_pos_rot(
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
