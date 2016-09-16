#!/usr/bin/env python

from subprocess import Popen, call
import signal

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty


class RelaunchVCI(object):
    def __init__(self):
        self.vci_proc = None
        self.vci_running = False
        self.start_vci_service = rospy.Service('/start_vci', Empty, self.start_vci)
        self.stop_vci_service = rospy.Service('/stop_vci', Empty, self.stop_vci)
        self.vci_status_pub = rospy.Publisher('/vci_running', Bool, latch=True, queue_size=1)
        self.vci_status_pub.publish(self.vci_running)
        rospy.loginfo("[%s] Relaunch VCI Service started.", rospy.get_name())

    def start_vci(self, req):
        if not self.vci_running:
            try:
                self.vci_proc = Popen(['roslaunch', 'assistive_teleop', 'vci.launch'])
                self.vci_running = True
            except:
                self.vci_proc.terminate()
                self.vci_running = False
            self.vci_status_pub.publish(self.vci_running)

    def stop_vci(self, req):
        if self.vci_running:
            if self.vci_proc is not None:
                self.vci_proc.terminate()
            self.vci_running = False
            self.vci_status_pub.publish(self.vci_running)


class RobotRestart(object):
    def __init__(self):
        self.robot_restart_service = rospy.Service('/robot_start', Empty, self.restart_robot_cb)
        rospy.loginfo("[%s] Robot Restart Service started.", rospy.get_name())

    def restart_robot_cb(self):
        rospy.loginfo("[%s] Request for robot start received. Restarting ROS now...", rospy.get_name())
        call(['robot', 'start', '-f'])


def main():
    rospy.init_node('software_reset_manager')
    vci_relaunch = RelaunchVCI()
    restart_handler = RobotRestart()
    rospy.spin()
