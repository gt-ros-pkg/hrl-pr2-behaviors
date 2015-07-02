#!/usr/bin/python

#This node is for casting ar_pose msgs to ar_track_alvar messages for use in hrl_pr2_ar_servo 
import roslib
roslib.load_manifest('hrl_pr2_ar_servo')
roslib.load_manifest('ar_track_alvar')

import rospy
import rostopic
from hrl_pr2_ar_servo.msg import ARServoGoalData
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from ar_pose.msg import ARMarker

from std_msgs.msg import Bool, Int8, String
from geometry_msgs.msg import PoseStamped

class recastMarkerMsg:
    def __init__(self):
        rospy.init_node('pr2_ar_servo_goal_config')
        self.defined=False      
        self.goal_pub=rospy.Publisher('ar_servo_goal_data', ARServoGoalData)
        rospy.Subscriber('ar_servo_goal_data', ARServoGoalData, self.goal_cb)
        
    def goal_cb(self, msg):
        goal=msg
        print "Got Message"
        if goal.marker_topic[0]=="/": #check if there is a '/' in front of the topic name so rostopic doesn't error
            marker_type, real_name, fn=rostopic.get_topic_type(goal.marker_topic)
        else:
            marker_type, real_name, fn=rostopic.get_topic_type("/"+goal.marker_topic)
        print (marker_type)
        self.marker_repub=rospy.Publisher('ar_tag_pose', AlvarMarkers)
 
        if marker_type == "ar_pose/ARMarker" and not self.defined:
            print"Republisher started for AR Pose markers"
            #Publish new goal message 
            self.send_new_goal(goal)          
            rospy.Subscriber(real_name, ARMarker, self.ar_pose_switch)
            self.defined=True
     
    def send_new_goal(self, goal):
        goal.marker_topic='ar_tag_pose'
        self.goal_pub.publish(goal)

    def ar_pose_switch(self, msg):
        #create and publish an Alvar message
        old_msg=msg
        new_msg=AlvarMarkers()
        new_msg.header=old_msg.header
        marker=AlvarMarker()
        marker.header=old_msg.header
        marker.id=old_msg.id
        marker.confidence=old_msg.confidence
        marker.pose.header=old_msg.header
        marker.pose.pose=old_msg.pose.pose
        new_msg.markers=[marker]
        self.marker_repub.publish(new_msg)
   
                
            
if __name__=="__main__":
    a=recastMarkerMsg()
    while not (rospy.is_shutdown()):
        rospy.spin()
