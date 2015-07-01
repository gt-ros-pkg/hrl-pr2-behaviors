#! /usr/bin/python

import numpy as np

import roslib
roslib.load_manifest("hrl_pr2_traj_playback")
import rospy
import rosparam
import roslaunch
from std_msgs.msg import Bool

from traj_playback import TrajPlayback
from traj_playback import CTRL_NAME_LOW, PARAMS_FILE_LOW
from msg import TrajPlaybackCmd

HEARTBEAT_TOPIC = '/arm_pose_move_gui/heartbeat'
COMMAND_TOPIC = '/arm_pose_move_gui/command'
MONITOR_RATE = 20
POSE_TRAJ_PARAM_FILE = 'pose_traj_dir.yaml'
POSE_TRAJ_PARAM_PREFIX = '$(find kelsey_sandbox)/params/'


class ArmPoseMoveGuiBackend(object):
    def __init__(self):
        params = rosparam.get_param("/arm_pose_move")
        self.pose_dict = params['poses']
        self.traj_dict = params['trajectories']
        self.apm_ctrl = TrajPlayback(CTRL_NAME_LOW, PARAMS_FILE_LOW)
        rospy.Subscriber(HEARTBEAT_TOPIC, Bool, self.heartbeat_cb)
        rospy.Subscriber(COMMAND_TOPIC, TrajPlaybackCmd, self.cmd_cb)

    def heartbeat_cb(self, msg):
        pass

    def cmd_cb(self, msg):
        if msg.type == msg.START:
            if not self.apm_ctrl.is_moving():
                if msg.is_trajectory:
                    filename = self.traj_dict[msg.traj_name]['file']
                    filepath = roslaunch.substitution_args.resolve_args(POSE_TRAJ_PARAM_PREFIX + filename)
                    if msg.is_setup:
                        result = self.apm_ctrl.move_to_setup_from_file(filepath,
                                                     reverse=not msg.is_forward, blocking=False)
                    else:
                        result = self.apm_ctrl.exec_traj_from_file(filepath,
                                                     reverse=not msg.is_forward, blocking=False)
                else:
                    filename = self.pose_dict[msg.traj_name]['file']
                    filepath = roslaunch.substitution_args.resolve_args(POSE_TRAJ_PARAM_PREFIX + filename)
        elif msg.type == msg.STOP:
            self.apm_ctrl.pause_moving()
        elif msg.type == msg.RESET:
            self.apm_ctrl.stop_moving()
        else:
            rospy.logerror("[arm_pose_move_backend] Bad command.")

def main():
    rospy.init_node("arm_pose_move_backend")
    apm_backend = ArmPoseMoveGuiBackend()
    rospy.spin()
