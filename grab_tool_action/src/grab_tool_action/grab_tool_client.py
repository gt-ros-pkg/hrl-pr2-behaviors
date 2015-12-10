#! /usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import *
import tf
from tf import transformations as tft
from grab_tool_action.msg import *
import numpy as np

class GrabToolClient (object): #sample client for grab_tool server
   
    def __init__ (self):
        self.client=actionlib.SimpleActionClient('grab_tool', GrabToolAction)
        rospy.Subscriber('grab_tool_server/feedback', GrabToolActionFeedback, self.feedback_cb)
        self.run()

    def feedback_cb(self, msg):
        feedback=msg
        print (feedback.feedback)                     
    
    def run (self):
        self.client.wait_for_server()
        print('done waiting for server')
        goal=GrabToolGoal()  #sample goal for grab_tool server
        goal.tag_topic='ar_marker_4'
        goal.arm='r'
        goal.pick=True
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result=self.client.get_result()


def main():
#if __name__=="__main__":
    rospy.init_node('grab_tool_client')
    GrabToolClient()
    while not rospy.is_shutdown():
        rospy.spin()
