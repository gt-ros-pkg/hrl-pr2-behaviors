#!/usr/bin/env python

from collections import deque
from copy import copy
import math

import roslib; roslib.load_manifest('ar_tool_grasp')
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import actionlib
from tf import transformations as tft
from pr2_controllers_msgs import Pr2GripperCommandAction, Pr2GripperCommandGoal

from ar_track_alvar.msg import AlvarMarker

from ar_tool_grasp.msg import ARToolGraspAction


class ARTagGraspAction(object):
    def __init__(self, setup_dist=0.1, tag_pose_window=1.0):
        #TODO: GET ALL THE MAGIC NUMBERS INTO PARAMETERS
        #TODO: Collect all magic strings into parameters
        self.setup_dist = setup_dist
        self.tag_pose_window = rospy.Duration(tag_pose_window)
        self.gripper_pose_sub = rospy.Subscriber('haptic_mpc/gripper_pose', PoseStamped, self.pose_cb)
        self.gripper_pose = None
        self.tag_poses = {}
        self.goal_pose_pub = rospy.Publisher('haptic_mpc/goal_pose', PoseStamped)
        self.gripper_ac = actionlib.SimpleActionClient('l_gripper_controller', Pr2GripperCommandAction)
        rospy.loginfo("[%s] Waiting for gripper action server")
        if self.gripper_ac.wait_for_server(rospy.Duration(10.0)):
            rospy.loginfo("[%s] Gripper action server started")
        else:
            rospy.logerr("[%s] Gripper action server not found")
        self.action_server = actionlib.SimpleActionServer('ARTagGraspAction', ARToolGraspAction, self.grasp_cb, False)
        self.action_server.start()

    def pose_cb(self, ps_msg):
        self.gripper_pose = ps_msg

    def clear_old_poses(self, pose_deque):
        now = rospy.Time.now()
        age = now - pose_deque[0]
        while age > self.tag_pose_window:
            pose_deque[marker.id].popleft()
            age = now - pose_deque[marker.id][0]
        return pose_deque

    def tag_pose_cb(self, marker_msg):
        for marker in marker_msg.markers:
            if not marker.id in self.tag_poses:
                pose = copy(marker.pose)
                pose.header = copy(marker.header)
                self.tag_poses[marker.id] = deque([pose])
            else:
                pose = copy(marker.pose)
                pose.header = copy(marker.header)
                self.tag_poses[marker.id].append(pose)
                self.tag_poses[marker.id] = self.clear_old_poses(self.tag_poses[marker.id])

    def get_current_tag_pose(self, tag_id):
        if (not tag_id in self.tag_poses) or
           (len(self.tag_poses[tag_id]) == 0):
            rospy.loginfo("[%] No pose available for tag id %d" %(rospy.get_name(), tag_id))
            return None
        else:
           N = len(self.tag_poses[tag_id])
           x = y = z = qx = qy = qz = qw = 0
           frame = self.tag_poses[tag_id][0].header.frame_id 
           for ps in self.tag_poses[tag_id]:
               if not(frame_id == ps.header.frame_id):
                   rospy.logerr("[%s] Frame Id's of stored poses for tag id %d do not match." %(rospy.get_name(), tag_id))
                   return None
               x += ps.pose.position.x
               y += ps.pose.position.y
               z += ps.pose.position.z
               qx += ps.pose.orientation.x
               qy += ps.pose.orientation.y
               qz += ps.pose.orientation.z
               qw += ps.pose.orientation.w
           x /= N
           y /= N
           z /= N
           q_mag = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
           qx /= q_mag
           qy /= q_mag
           qz /= q_mag
           qw /= q_mag
           ps = PoseStamped()
           ps.header.frame_id = frame
           ps.header.stamp = rospy.Time.now()
           ps.pose.position = Point(x,y,z)
           ps.pose.orientation = Quaternion(qx, qy, qz, qw)
           return ps

    def goal_from_tag_pose(self, tag_pose):
        #TODO: FIX TRANSFORM AXES!!!
        transform = self.tag_to_goal_transform
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = tag_pose.frame_id
        goal_pose.pose = copy(tag_pose.pose)
        goal_pose.pose.position.z -= 0.02
        rot = tft.quaternion_from_euler(0, math.pi/2., 0.)
        q = (goal_pose.pose.orientation.x,
             goal_pose.pose.orientation.y,
             goal_pose.pose.orientation.z,
             goal_pose.pose.orientation.w)
        q_new =  tft.quaternion_multiply(q,rot)
        goal_pose.pose.orientation = Quaternion(*q_new)
        return goal_pose

    def setup_from_tag_pose(self, tag_pose):
        #TODO: FIX TRANSFORM AXES!!!
        transform = self.setup_to_goal_transform
        tf_mat = tft.compose_matrix(angles=transform[3:], translate=transform[:3])
        tag_xyz = (tag_pose.pose.position.x,
                   tag_pose.pose.position.y,
                   tag_pose.pose.position.z)
        tag_rpy
        tag_mat

        setup_pose = PoseStamped()
        setup_pose.header.frame_id = tag_pose.frame_id
        setup_pose.pose = copy(tag_pose.pose)
        setup_pose.pose.position.z += self.setup_dist
        rot = tft.quaternion_from_euler(0, math.pi/2., 0.)
        q = (setup_pose.pose.orientation.x,
             setup_pose.pose.orientation.y,
             setup_pose.pose.orientation.z,
             setup_pose.pose.orientation.w)
        q_new =  tft.quaternion_multiply(q,rot)
        setup_pose.pose.orientation = Quaternion(*q_new)
        return setup_pose

    def open_gripper(self, dist=0.09, effort=-1):
        gripper_goal = Pr2GripperCommandGoal()
        gripper_goal.command.position = dist
        gripper_goal.command.effort = effort
        self.gripper_ac.send_goal(gripper_goal)

    def close_gripper(self, dist=0.0, effort=-1):
        gripper_goal = Pr2GripperCommandGoal()
        gripper_goal.command.position = dist
        gripper_goal.command.effort = effort
        self.gripper_ac.send_goal(gripper_goal)

    def get_pose_error(self, goal_pose, curr_pose):

        return pos_err, rot_err

    def progressing(self):
        pos_err, ort_err = self.get_pose_error(self.goal_pose, self.gripper_pose)
        now = rospy.Time.now()
        if self.pos_err <= 0.9 * self.last_pos_err:
            self.last_pos_err = pos_err
            self.last_progress_time = now
            return True
        if self.ort_err <= 0.9 * self.last_ort_err:
            self.last_ort_err = ort_err
            self.last_progress_time = now
            return True
        if now - self.last_progress_time > self.progress_timeout:
            return False
        return True

    def grasp_cb(self, grasp_goal):
        tag_id = grasp_goal.tag_id
        tag_pose = self.get_current_tag_pose(grasp_goal.tag_id)
        if tag_pose is None:
            self.set_aborted(None, "Tag not Seen")
            return
        rospy.loginfo("[%s] Grasping tag id: %d" %(rospy.get_name(), tag_id))
        self.goal_pose = self.setup_from_tag_pose(tag_pose)
        self.open_gripper()
        self.goal_pose_pub.publish(self.goal_pose)
        self.pos_err, self.ort_err = self.get_pose_error(self.goal_pose, self.gripper_pose)
        while (self.pos_err > 0.03) and (self.ort_err > math.pi/12):
            if self.progressing()
                rospy.sleep(0.5)
            else:
                self.set_aborted(None, "Not progressing toward goal...")
        result = self.gripper_ac.get_result()
        if not result.position > 0.08:
            self.fail("Gripper Not Open")

        goal_pose = self.goal_from_tag_pose(tag_pose)
        self.goal_pose_pub.publish(goal_pose)
        self.pos_err, self.ort_err = self.get_pose_error(setup_pose, self.gripper_pose)
        while (self.pos_err > 0.01) and (self.ort_err > math.pi/18):
            if self.progressing(self.pos_err, self.ort_err):
                rospy.sleep(0.5)
            else:
                self.set_aborted(None, "Not progressing toward goal...")
       self.close_gripper(dist=0.010)
       closed = self.gripper_ac.wait_for_result(15.0)
       if not closed:
           self.set_aborted(None, "Couldn't close gripper")
       result = self.gripper_ac.get_result()
       if (0.035 > result.position > 0.02):
           self.action_server.set_succeeded(self, result, text=msg)


if __name__=='__main__':
    rospy.init_node('ar_tag_grasp_action')
    grasp_action = ARTagGraspAction(tag_id=0, setup_dist = 0.1)
    rospy.spin()

