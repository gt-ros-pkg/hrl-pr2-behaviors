#!/usr/bin/env python


import rospy
import roslaunch
from std_msgs.msg import Bool
from std_srvs.srv import Empty


VCI_PATH = '/home/pgrice/catkin_ws/src/hrl-assistive/assistive_teleop/launch/vci.launch'


class RelaunchVCI(object):
    def __init__(self):
        self.vci_uuid = roslaunch.rlutil.get_or_make_uuid(None, False)
        roslaunch.configure_logging(self.vci_uuid)
        self.vci_launch = roslaunch.parent.ROSLaunchParent(self.vci_uuid, [VCI_PATH])
        self.vci_running = False
        self.start_vci_service = rospy.Service('/start_vci', Empty, self.start_vci)
        self.stop_vci_service = rospy.Service('/stop_vci', Empty, self.stop_vci)
        self.vci_status_pub = rospy.Publisher('/vci_running', Bool, latch=True, queue_size=1)
        self.vci_status_pub.publish(self.vci_running)

    def start_vci(self):
        if not self.vci_running:
            self.vci_launch.start()
            self.vci_running = True
            self.vci_status_pub.publish(self.vci_running)

    def stop_vci(self):
        if self.vci_running:
            self.vci_launch.shutdown()
            self.vci_running = False
            self.vci_status_pub.publish(self.vci_running)


from subprocess import call


class RobotRestart(object):
    def __init__(self):
        self.robot_restart_service = rospy.Service('/robot_start', Empty, self.restart_robot_cb)

    def restart_robot_cb(self):
        rospy.loginfo("[%s] Request for robot start received. Restarting ROS now...", rospy.get_name())
        call(['robot', 'start', '-f'])


def main():
    rospy.init_node('software_reset_manager')
    vci_relaunch = RelaunchVCI()
    restart_handler = RobotRestart()
    rospy.spin()
