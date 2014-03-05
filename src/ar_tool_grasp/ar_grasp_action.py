#!/usr/bin/env python

import sys

from collections import deque
from copy import copy
import numpy as np

import roslib; roslib.load_manifest('ar_tool_grasp')
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
from tf import transformations as tft

from ar_track_alvar.msg import AlvarMarkers

from ar_tool_grasp.msg import ARToolGraspAction, ARToolGraspFeedback, ARToolGraspResult

class ARTagGraspAction(object):
    def __init__(self, tag_pose_window=1.0, progress_timeout=5.,
                 setup_transform=None, grasp_transform=None):
        #TODO: GET ALL THE MAGIC NUMBERS INTO PARAMETERS
        #TODO: Collect all magic strings into parameters
        self.tag_pose_window = rospy.Duration(tag_pose_window)
        self.progress_timeout = rospy.Duration(progress_timeout)
        self.setup_transform = setup_transform if setup_transform is not None else [0]*6
        self.grasp_transform = grasp_transform if grasp_transform is not None else [0]*6

        self.gripper_pose = None
        self.gripper_pose_sub = rospy.Subscriber('haptic_mpc/gripper_pose', PoseStamped, self.pose_cb)

        self.tag_poses = {}
        self.tag_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.tag_pose_cb)

        self.goal_pose_pub = rospy.Publisher('haptic_mpc/goal_pose', PoseStamped)
        self.test_pub_1 = rospy.Publisher('test_pose_1', PoseStamped, latch=True)
        self.test_pub_2 = rospy.Publisher('test_pose_2', PoseStamped, latch=True)
        self.last_pos_err = self.last_ort_err = np.inf
        self.last_progress_time = rospy.Time.now()
        self.gripper_ac = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        rospy.loginfo("[%s] Waiting for gripper action server" %(rospy.get_name()))
        if self.gripper_ac.wait_for_server(rospy.Duration(10.0)):
            rospy.loginfo("[%s] Gripper action server started" %(rospy.get_name()))
        else:
            rospy.logerr("[%s] Gripper action server not found" %(rospy.get_name()))
        self.action_server = actionlib.SimpleActionServer('ar_tool_grasp_action', ARToolGraspAction, self.grasp_cb, False)
        self.action_server.start()
        rospy.loginfo("[%s] AR Tool Grasp Action Server Started" %(rospy.get_name()))

    def pose_cb(self, ps_msg):
        self.gripper_pose = ps_msg

    def clear_old_poses(self, pose_deque):
        now = rospy.Time.now()
        age = now - pose_deque[0].header.stamp
        while age > self.tag_pose_window:
            pose_deque.popleft()
            age = now - pose_deque[0].header.stamp
        return pose_deque

    def tag_pose_cb(self, markers_msg):
        for marker in markers_msg.markers:
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
        if ((not tag_id in self.tag_poses) or
            (len(self.tag_poses[tag_id]) == 0)):
            rospy.loginfo("[%s] No pose available for tag id %d" %(rospy.get_name(), tag_id))
            return None
        else:
           N = len(self.tag_poses[tag_id])
           x = y = z = qx = qy = qz = qw = 0
           frame = self.tag_poses[tag_id][0].header.frame_id
           for ps in self.tag_poses[tag_id]:
               if not(frame == ps.header.frame_id):
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
           q_mag = np.linalg.norm((qx, qy, qz, qw))
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
        tf_mat = tft.compose_matrix(angles=self.grasp_transform[3:])
        tag_q = (tag_pose.pose.orientation.x, tag_pose.pose.orientation.y,
                    tag_pose.pose.orientation.z, tag_pose.pose.orientation.w)
        tag_mat = tft.quaternion_matrix(tag_q)
        new_mat = np.dot(tag_mat, tf_mat)
        dxyz = self.grasp_transform[:3]
        dxyz.append(1)
        d_xyz_base = np.matrix(dxyz).T
        d_xyz_tag = np.dot(tag_mat, d_xyz_base).A1[:3].tolist()
        new_q = tft.quaternion_from_matrix(new_mat)
        grasp_pose = PoseStamped()
        grasp_pose.header.stamp = rospy.Time.now()
        grasp_pose.header.frame_id = tag_pose.header.frame_id
        grasp_pose.pose.position.x = tag_pose.pose.position.x + d_xyz_tag[0]
        grasp_pose.pose.position.y = tag_pose.pose.position.y + d_xyz_tag[1]
        grasp_pose.pose.position.z = tag_pose.pose.position.z + d_xyz_tag[2]
        grasp_pose.pose.orientation = Quaternion(*new_q)
        self.test_pub_2.publish(grasp_pose)
        return grasp_pose

    def setup_from_tag_pose(self, tag_pose):
        print tag_pose
        tf_mat = tft.compose_matrix(angles=self.setup_transform[3:])
        tag_q = (tag_pose.pose.orientation.x, tag_pose.pose.orientation.y,
                 tag_pose.pose.orientation.z, tag_pose.pose.orientation.w)
        tag_mat = tft.quaternion_matrix(tag_q)
        new_mat = np.dot(tag_mat, tf_mat)
        dxyz = self.setup_transform[:3]
        dxyz.append(1)
        d_xyz_base = np.matrix(dxyz).T
        d_xyz_tag = np.dot(tag_mat, d_xyz_base).A1[:3].tolist()
        new_q = tft.quaternion_from_matrix(new_mat)
        setup_pose = PoseStamped()
        setup_pose.header.stamp = rospy.Time.now()
        setup_pose.header.frame_id = tag_pose.header.frame_id
        setup_pose.pose.position.x = tag_pose.pose.position.x + d_xyz_tag[0]
        setup_pose.pose.position.y = tag_pose.pose.position.y + d_xyz_tag[1]
        setup_pose.pose.position.z = tag_pose.pose.position.z + d_xyz_tag[2]
        setup_pose.pose.orientation = Quaternion(*new_q)
        print setup_pose
        self.test_pub_1.publish(setup_pose)
        return setup_pose

    def open_gripper(self, dist=0.09, effort=-1):
        gripper_goal = Pr2GripperCommandGoal()
        gripper_goal.command.position = dist
        gripper_goal.command.max_effort = effort
        self.gripper_ac.send_goal(gripper_goal)

    def close_gripper(self, dist=0.0, effort=-1):
        gripper_goal = Pr2GripperCommandGoal()
        gripper_goal.command.position = dist
        gripper_goal.command.max_effort = effort
        self.gripper_ac.send_goal(gripper_goal)

    def get_pose_error(self, goal_pose, curr_pose):
        dx = curr_pose.pose.position.x - goal_pose.pose.position.x
        dy = curr_pose.pose.position.y - goal_pose.pose.position.y
        dz = curr_pose.pose.position.z - goal_pose.pose.position.z
        pos_err = np.linalg.norm((dx, dy, dz))
        goal_q = (goal_pose.pose.orientation.x,
                  goal_pose.pose.orientation.y,
                  goal_pose.pose.orientation.z,
                  goal_pose.pose.orientation.w)
        curr_q = (curr_pose.pose.orientation.x,
                  curr_pose.pose.orientation.y,
                  curr_pose.pose.orientation.z,
                  curr_pose.pose.orientation.w)
        #goal_q = curr_q * delta_q -> delta_q = curr_q^-1 * goal_q
        cq_inv = tft.quaternion_inverse(curr_q)
        dq = tft.quaternion_multiply(cq_inv, goal_q)
        #theta = 2*arctan2( ||X||, w)
        q_vec_norm = np.linalg.norm(dq[:3])
        rot_err = 2*np.arctan2(q_vec_norm, dq[3])
        return pos_err, rot_err

    def progressing(self):
        self.pos_err, self.ort_err = self.get_pose_error(self.goal_pose, self.gripper_pose)
        now = rospy.Time.now()
        if self.pos_err <= 0.9 * self.last_pos_err:
            self.last_pos_err = self.pos_err
            self.last_progress_time = now
            return True
        if self.ort_err <= 0.9 * self.last_ort_err:
            self.last_ort_err = self.ort_err
            self.last_progress_time = now
            return True
        if now - self.last_progress_time > self.progress_timeout:
            return False
        return True

    def grasp_cb(self, grasp_goal):
        tag_id = grasp_goal.tag_id
        tag_pose = self.get_current_tag_pose(grasp_goal.tag_id)
        result = ARToolGraspResult()
        result.succeeded = False
        if tag_pose is None:
            result.final_pose = copy(self.gripper_pose)
            msg = "[%s] Failed to grasp tag id %d: Location of tag unknown." %(rospy.get_name(), tag_id)
            rospy.loginfo(msg)
            self.action_server.set_aborted(result, msg)
            return
        rospy.loginfo("[%s] Grasping tag id: %d" %(rospy.get_name(), tag_id))
        self.goal_pose = self.setup_from_tag_pose(tag_pose)
        self.open_gripper()
        self.goal_pose_pub.publish(self.goal_pose)
        self.pos_err, self.ort_err = self.get_pose_error(self.goal_pose, self.gripper_pose)
        while (self.pos_err > 0.03) and (self.ort_err > np.pi/18):
            print "Pos err: ", self.pos_err
            print "Ort Err: ", self.ort_err
            if self.progressing():
                fdbk = ARToolGraspFeedback()
                fdbk.progressing = True
                self.action_server.publish_feedback(fdbk)
                rospy.sleep(0.5)
                print "Approaching setup"
            else:
                result.final_pose = copy(self.gripper_pose)
                msg = "[%s] Failed to grasp tag id %d: Could not reach setup pose." %(rospy.get_name(), tag_id)
                rospy.loginfo(msg)
                self.action_server.set_aborted(result, msg)
                return
        print "Reached setup"
        gripper_result = self.gripper_ac.get_result()
        if not gripper_result.position > 0.08:
            result.final_pose = copy(self.gripper_pose)
            msg = "[%s] Failed to grasp tag id %d: Could not open gripper." %(rospy.get_name(), tag_id)
            rospy.loginfo(msg)
            self.action_server.set_aborted(result, msg)
            return

        goal_pose = self.goal_from_tag_pose(tag_pose)
        self.goal_pose_pub.publish(goal_pose)
        self.pos_err, self.ort_err = self.get_pose_error(self.goal_pose, self.gripper_pose)
        while (self.pos_err > 0.01) and (self.ort_err > np.pi/18):
            if self.progressing():
                fdbk = ARToolGraspFeedback()
                fdbk.progressing = True
                self.action_server.publish_feedback(fdbk)
                rospy.sleep(0.5)
                print "Approaching grasp"
            else:
                result.final_pose = copy(self.gripper_pose)
                msg = "[%s] Failed to grasp tag id %d: Could not reach grasp pose." %(rospy.get_name(), tag_id)
                rospy.loginfo(msg)
                self.action_server.set_aborted(result, msg)
                return
        print "Reached Grasp"
        self.close_gripper(dist=0.010)
        closed = self.gripper_ac.wait_for_result(rospy.Duration(15.0))
        if not closed:
            result.final_pose = copy(self.gripper_pose)
            msg = "[%s] Failed to grasp tag id %d: Could not close gripper." %(rospy.get_name(), tag_id)
            rospy.loginfo(msg)
            self.action_server.set_aborted(result, msg)
            return
        result = self.gripper_ac.get_result()
        if (0.035 > result.position > 0.02):
            result = ARToolGraspResult()
            result.succeeded = True
            result.final_pose = copy(self.gripper_pose)
            msg = "[%s] Succeeded in grasping tag %d" %(rospy.get_name(), tag_id)
            rospy.loginfo(msg)
            self.action_server.set_succeeded(result, text=msg)

if __name__=='__main__':
    import argparse
    parser = argparse.ArgumentParser(description="A node for grasping a tool handle marked with an AR Tag",
                                     formatter_class = argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-s','--setup-transform', action='store', nargs=6, type=float, default=[0.]*6,
                        help="Setup Transform: A pose (x,y,z,r,p,y) relative to the tag face which should be used as an approach position.")
    parser.add_argument('-g','--grasp-transform', action='store', nargs=6, type=float, default=[0.]*6,
                        help="Grasp Transform: A pose (x,y,z,r,p,y) relative to the tag face which should be used as the grasping location.")
    parser.add_argument('-t', '--timeout', action="store", type=float, default=5.0,
                        help="Duration in seconds to wait for 5 percent improvement in error before declaring failure.")
    parser.add_argument('-w', '--window', action="store", type=float, default=1.0,
                        help="Duration in seconds of windowed average of tag pose.")
    args = parser.parse_known_args()[0]

    rospy.init_node('ar_tag_grasp_action')
    args.setup_transform[3:] = np.radians(args.setup_transform[3:])
    args.grasp_transform[3:] = np.radians(args.grasp_transform[3:])
    grasp_action = ARTagGraspAction(tag_pose_window=args.window,
                                    progress_timeout=args.timeout,
                                    setup_transform=args.setup_transform,
                                    grasp_transform=args.grasp_transform)
    rospy.spin()


