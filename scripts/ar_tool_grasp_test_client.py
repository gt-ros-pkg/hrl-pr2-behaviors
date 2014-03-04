#!/usr/bin/env python

import sys

import roslib; roslib.load_manifest('ar_tool_grasp')
import rospy
import actionlib

from ar_tool_grasp.msg import ARToolGraspAction, ARToolGraspGoal

class ARToolGraspTestClient(object):
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('ar_tool_grasp_action', ARToolGraspAction)
        rospy.loginfo("[%s] Waiting for ar_tool_grasp_action server" %(rospy.get_name()))
        if self.action_client.wait_for_server(rospy.Duration(10.0)):
            rospy.loginfo("[%s] ar_tool_grasp_action server started" %(rospy.get_name()))
        else:
            rospy.logerr("[%s] ar_tool_grasp_action server not found" %(rospy.get_name()))
            sys.exit()

    def grasp_tag(self, tag_id):
        goal = ARToolGraspGoal()
        goal.tag_id = tag_id
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        res = self.action_client.get_result()
        if res.succeeded:
            rospy.loginfo("[%s] Grasp of Tool %d Successful!" %(rospy.get_name(), tag_id))
        else:
            rospy.logwarn("[%s] Grasp of Tool %d Failed!" %(rospy.get_name(), tag_id))


if __name__=='__main__':
    rospy.init_node('ar_tool_grasp_test_client')
    test_client = ARToolGraspTestClient()
    for tag_id in sys.argv[1:]:
        rospy.loginfo("[%s] Testing grasping tag id %s" %(rospy.get_name(), tag_id))
        test_client.grasp_tag(int(tag_id))
        rospy.loginfo("[%s] Test completed for tag id %s" %(rospy.get_name(), tag_id))
    rospy.loginfo("[%s] All tests completed" %(rospy.get_name()))
