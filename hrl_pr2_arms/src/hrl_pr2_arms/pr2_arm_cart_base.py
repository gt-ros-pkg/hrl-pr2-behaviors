#! /usr/bin/python
#
# Base class for interacting with a PR2 Cartesian realtime controller.
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
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

from ep_arm_base import EPArmBase, create_ep_arm
from hrl_geom.pose_converter import PoseConv
import hrl_geom.transformations as trans

def extract_twist(msg):
    return np.array([msg.linear.x, msg.linear.y, msg.linear.z, 
                     msg.angular.x, msg.angular.y, msg.angular.z])

##
# Base class for interacting with the Cartesian controllers on the PR2.
# The equilibrium points are pose-like objects.
class PR2ArmCartBase(PR2Arm):
    def __init__(self, arm_side, urdf, base_link='torso_lift_link', end_link='%s_gripper_tool_frame', 
                 controller_name='/%s_cart', kdl_tree=None, timeout=1.):
        super(PR2ArmCartBase, self).__init__(arm_side, urdf, base_link, end_link, 
                                             controller_name, kdl_tree, timeout)
        self.command_pose_pub = rospy.Publisher(self.controller_name + '/command_pose', PoseStamped)

    ##
    # Command the realtime controller to set its cartesian equilibrium
    # point to this pose.
    # @param Pose-like object
    def set_ep(self, pose):
        cep_pose_stmp = PoseConv.to_pose_stamped_msg('torso_lift_link', pose)
        self.command_pose_pub.publish(cep_pose_stmp)

    ##
    # Returns pairs of positions and rotations linearly interpolated between
    # the start and end position/orientations.  Rotations are found using slerp
    # @return List of (pos, rot) waypoints between start and end.
    def interpolate_ep(self, ep_a, ep_b, t_vals):
        pos_a, rot_a = ep_a
        pos_b, rot_b = ep_b
        num_samps = len(t_vals)
        pos_waypoints = (np.array(pos_a) + 
                         np.array(np.tile(pos_b - pos_a, (1, num_samps))) * np.array(t_vals)
        pos_waypoints = [np.mat(pos).T for pos in pos_waypoints.T]
        rot_homo_a, rot_homo_b = np.eye(4), np.eye(4)
        rot_homo_a[:3,:3] = rot_a
        rot_homo_b[:3,:3] = rot_b
        quat_a = trans.quaternion_from_matrix(rot_homo_a)
        quat_b = trans.quaternion_from_matrix(rot_homo_b)
        rot_waypoints = []
        for t in t_vals:
            cur_quat = trans.quaternion_slerp(quat_a, quat_b, t)
            rot_waypoints.append(np.mat(trans.quaternion_matrix(cur_quat))[:3,:3])
        return zip(pos_waypoints, rot_waypoints)

    ##
    # Gets the Cartesian difference in the two poses.
    def cart_error(self, ep_actual, ep_desired):
        pos_act, rot_act = PoseConv.to_pos_rot(ep_actual)
        pos_des, rot_des = PoseConv.to_pos_rot(ep_desired)
        err = np.mat(np.zeros((6, 1)))
        err[:3,0] = pos_act - pos_des
        err[3:6,0] = np.mat(trans.euler_from_matrix(rot_des.T * rot_act)).T
        return err

##
# Class which extends the PR2ArmCartBase to provide functionality for changing the
# posture.
class PR2ArmCartPostureBase(PR2ArmCartBase):
    def __init__(self, arm_side, kinematics, controller_name='/%s_cart', timeout=5.):
        super(PR2ArmCartPostureBase, self).__init__(arm_side, kinematics, controller_name, timeout)
        self.command_posture_pub = rospy.Publisher(self.controller_name + '/command_posture', 
                                                   Float64MultiArray)

    ##
    # Sets the null-space posture the arm will attempt to reach with its
    # extra degree of freedom
    # @param List of joint angles representing the posture.  If any element is None,
    #        posture control is disabled for that joint. If posture is None, posture
    #        control is disabled for all joints.
    def set_posture(self, posture=None):
        if posture is None:
            posture = [None] * len(self.get_joint_names())
        posture = copy.copy(posture)
        for i, p in enumerate(posture):
            if p is None:
                posture[i] = 9999
        msg = Float64MultiArray()
        msg.data = posture
        self.command_posture_pub.publish(msg)

