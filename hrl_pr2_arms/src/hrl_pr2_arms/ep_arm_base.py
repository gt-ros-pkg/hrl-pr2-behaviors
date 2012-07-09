#! /usr/bin/python
#
# Base class for interacting with a realtime controller using an equilibrium point control
# framework.
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

from threading import Lock
import numpy as np

import roslib
roslib.load_manifest('hrl_pr2_arms')

import rospy

from urdf_parser_py.urdf import URDF
from pykdl_utils.joint_kinematics import JointKinematics


##
# Base class for interacting with a realtime controller using an equilibrium point control
# frame work.  An equilibrium point is a set of parameters representing the state of the
# controller at which no torques would be applied. For a PID controller, this represents
# the joint angles where the error would be zero.
class EPArmBase(JointKinematics):
    ##
    # Constructor
    # @param arm_side Used to signify side of robot for similar arms 
    #                 (e.g. 'r' for right, 'l' for left)
    # @param urdf URDF object of robot.
    # @param base_link Name of the root link of the kinematic chain. 
    #                  Will fill in arm_side if %s is in string.
    # @param end_link Name of the end link of the kinematic chain.
    #                 Will fill in arm_side if %s is in string.
    # @param controller_name Optional name of the controller which will be subscribed to
    #                        to obtain controller information
    # @param kdl_tree Optional KDL.Tree object to use. If None, one will be generated
    #                          from the URDF.
    # @param timeout Time in seconds to wait for the /joint_states topic.
    def __init__(self, arm_side, urdf, base_link, end_link, controller_name=None, 
                 kdl_tree=None, timeout=1.):
        if "%s" in base_link:
            base_link = base_link % arm_side
        if "%s" in end_link:
            end_link = end_link % arm_side
        if controller_name is not None and "%s" in controller_name:
            controller_name = controller_name % arm_side
        super(EPArmBase, self).__init__(urdf, base_link, end_link, kdl_tree, timeout)
        self.arm_side = arm_side
        self.controller_name = controller_name
        self.ep = None # equilibrium point
        self.ep_time = None # time ep was last recorded
        self.ep_lock = Lock()
        self.ctrl_state_lock = Lock()
        self.ctrl_state_dict = {} # stores information from the realtime controller

    ##
    # Returns the current pose of the tooltip
    # @return 4x4 np.mat homogeneous matrix
    def get_end_effector_pose(self):
        return self.forward()

    ##
    # Returns the current equilibrium point as read from the controller
    # @return equilibrium point
    def get_ep(self):
        with self.ep_lock:
            if self.ep is None:
                rospy.logwarn("[ep_arm_base] Equilibrium point not read yet.")
            return copy.copy(self.ep)

    ##
    # Commands the arm to move to desired equilbrium point
    def set_ep(self, *args):
        raise RuntimeError('Unimplemented Function')

    ##
    # Wait until we have the ep from the controller
    # @param timeout Time in seconds at which we break if we haven't recieved the EP.
    def wait_for_ep(self, timeout=1.):
        start_time = rospy.get_time()
        r = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.get_time() - start_time < timeout:
            with self.ep_lock:
                if self.ep is not None:
                    return True
            r.sleep()
        return False
            
    ##
    # Returns the current state of the realtime controller. The state is filled in
    # by the subclasses.
    # @return dict with realtime controller information.
    def get_controller_state(self):
        with self.ctrl_state_lock:
            if self.controller_name is None:
                rospy.logerror("[ep_arm_base] get_controller_state NOT IMPLEMENTED!")
                return None
            elif len(self.ctrl_state_dict) == 0:
                rospy.logwarn("[ep_arm_base] Controller state not yet published.")
                return None
            return self.ctrl_state_dict

    ##
    # Returns whether the equilibrium point has been recorded for some time.
    # @param stale_time Time in seconds to pass to consider the EP stale.
    # @return True if the EP is stale.
    def is_ep_stale(self, stale_time=1.):
        with self.ep_lock:
            if self.ep is None:
                rospy.logwarn("[ep_arm_base] Equilibrium point not read yet.")
                return True
            return rospy.get_time() - self.ep_time > stale_time

    ##
    # Method to be implemented by subclasses to record the equilibrium point.
    def _save_ep(self, ep):
        with self.ep_lock:
            self.ep_time = rospy.get_time()
            self.ep = ep

def create_ep_arm(arm_side, arm_type=EPArmBase, base_link="torso_lift_link",  
                  end_link="%s_gripper_tool_frame", urdf_filename=None,
                  controller_name=None, timeout=5.):
    if urdf_filename is None:
        robot = URDF.load_from_parameter_server(verbose=False)
    else:
        robot = URDF.load_xml_file(urdf_filename, verbose=False)
    return arm_type(arm_side, robot, base_link=base_link, end_link=end_link,
                    controller_name=controller_name, timeout=timeout)
