#!/usr/bin/env python

import numpy as np
from threading import Thread

import roslib
roslib.load_manifest('hrl_pr2_ar_servo')
roslib.load_manifest('smach_ros')
import rospy
import smach
from std_msgs.msg import Bool, Int8
from geometry_msgs.msg import PoseStamped
from tf import transformations as tft
import tf

from pykdl_utils.joint_kinematics import create_joint_kin
from pr2_ar_servo import PR2ARServo

from hrl_pr2_ar_servo.msg import ARServoGoalData

OUTCOMES_SPA = ['succeeded', 'preempted', 'aborted']


class ServoStates:
    BEGIN_FIND_TAG = 1
    FOUND_TAG = 2
    TIMEOUT_FIND_TAG = 3
    BEGIN_SERVO = 4
    SUCCESS_SERVO = 5
    ARM_COLLISION = 6
    LASER_COLLISION = 7
    LOST_TAG = 8
    USER_PREEMPT = 9


class ArmCollisionDetection(smach.State):
    def __init__(self,
                 min_l_torques=[-5.]*7, min_r_torques=[-5.]*7,
                 max_l_torques=[5.]*7, max_r_torques=[5.]*7):
        smach.State.__init__(self, outcomes=['collision', 'preempted', 'aborted'])
        self.min_l_tor = np.array(min_l_torques)
        self.min_r_tor = np.array(min_r_torques)
        self.max_l_tor = np.array(max_l_torques)
        self.max_r_tor = np.array(max_r_torques)
        self.l_arm = create_joint_kin('torso_lift_link', 'l_gripper_tool_frame')
        self.r_arm = create_joint_kin('torso_lift_link', 'r_gripper_tool_frame')

    def execute(self, userdata):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            l_tor = self.l_arm.get_joint_efforts()
            r_tor = self.r_arm.get_joint_efforts()
            if np.any(l_tor < self.min_l_tor):
                print "Collision detected on left arm with torque:", l_tor
                print "Minimum torques:", self.min_l_tor
                return 'collision'
            if np.any(l_tor > self.max_l_tor):
                print "Collision detected on left arm with torque:", l_tor
                print "Maximum torques:", self.max_l_tor
                return 'collision'
            if np.any(r_tor < self.min_r_tor):
                print "Collision detected on right arm with torque:", r_tor
                print "Minimum torques:", self.min_r_tor
                return 'collision'
            if np.any(r_tor > self.max_r_tor):
                print "Collision detected on right arm with torque:", r_tor
                print "Minimum torques:", self.max_r_tor
                return 'collision'

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            r.sleep()
        return 'aborted'

# class LaserCollisionDetection(smach.State):
#    def __init__(self):
#         smach.State.__init__(self, outcomes=['collision', 'preempted', 'aborted'])
#         self.cs = CostmapServices(accum=3)
#
#     def execute(self, userdata):
#         r = rospy.Rate(20)
#         while not rospy.is_shutdown():
#             vx, vy, vtheta = 0., 0., 0. #TODO REMOVE
#             score = self.cs.scoreTraj_PosHyst(vx, vy, vtheta) # TODO
#             if score < 0:
#                 print "Base laser detected a collision."
#                 return 'collision'
#             if self.preempt_requested():
#                 self.service_preempt()
#                 return 'preempted'
#             r.sleep()
#         return 'aborted'


class BoolTopicState(smach.State):
    def __init__(self, topic, rate=20):
        smach.State.__init__(self, outcomes=['true', 'false', 'preempted', 'aborted'])
        self.rate = rate
        self.ui_list = rospy.Subscriber(topic, Bool, self.ui_cb)

    def ui_cb(self, msg):
        self.bool_msg = msg.data
        self.got_msg = True

    def execute(self, userdata):
        self.got_msg = False
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.got_msg:
                if self.bool_msg:
                    return 'true'
                else:
                    return 'false'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            r.sleep()
        return 'aborted'


class PublishState(smach.State):
    def __init__(self, topic, msg_type, msg):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub = rospy.Publisher(topic, msg_type, queue_size=1)
        self.msg = msg

    def execute(self, userdata):
        self.pub.publish(self.msg)
        return 'succeeded'


class FindARTagState(smach.State):
    """ A Smach State for finding an AR Tag in a given camera topic. """
    def __init__(self, viz_servo, timeout=None):
        smach.State.__init__(self,
                             outcomes = ["found_tag", "timeout", "preempted", "aborted"],
                             input_keys=["tag_id"],
                             output_keys=["initial_ar_pose"])
        self.timeout = timeout
        self.viz_servo = viz_servo

    def execute(self, userdata):
        """ Wait until tag is found, or timeout."""
        def check_preempt(event):
            if self.preempt_requested():
                self.service_preempt()
                self.viz_servo.request_preempt()
        preempt_timer = rospy.Timer(rospy.Duration(0.1), check_preempt)
        #TODO: Modify find_ar_tag to accept a tag id for greater specificity
        tag, result = self.viz_servo.find_ar_tag(self.timeout)
        print "Find ar tag result: ", result
        if result in ["preempted", "aborted", "timeout"]:
            return result
        elif result == "found_tag":
            userdata["initial_ar_pose"] = tag
            return "found_tag"

class ServoARTagState(smach.State):
    """ A Smach State for servoing to an AR Tag. """
    def __init__(self, viz_servo, tag_goal_xyt):
        smach.State.__init__(self, outcomes=['lost_tag'] + OUTCOMES_SPA,
                                   input_keys=['initial_ar_pose'])
        self.viz_servo = viz_servo
        self.tag_goal_xyt = tag_goal_xyt

    def execute(self, userdata):
        """ Servo to a goal AR Tag, reporting success or lost tag, which checking for preemption."""
        def check_preempt(event):
            if self.preempt_requested():
                self.service_preempt()
                self.viz_servo.request_preempt()
        preempt_timer = rospy.Timer(rospy.Duration(0.1), check_preempt)

        initial_ar_pose = userdata['initial_ar_pose'] if 'initial_ar_pose' in userdata else None
        self.viz_servo.preempt_requested = False
        outcome = self.viz_servo.servo_to_tag(pose_goal=self.tag_goal_xyt, initial_ar_pose=initial_ar_pose)
        preempt_timer.shutdown()
        return outcome

class ServoOnTagGoal(Thread):
    """ A Smach State that waits for a msg.
        Msg should contain the goal location, tag id(if any), and camera topic for AR Servoing.
    """
    def __init__(self, marker_topic, tag_goal_xyt, tag_id, find_tag_timeout=None):
        Thread.__init__(self)
        self.tag_goal_xyt = tag_goal_xyt
        self.viz_servo = PR2ARServo(marker_topic, tag_id)
        self.sm_pr2_servoing = self.build_full_sm(self.viz_servo, self.tag_goal_xyt, find_tag_timeout=None)
        self.daemon = True

    def preempt(self):
        return self.sm_pr2_servoing.request_preempt()

    def run(self):
        self.sm_pr2_servoing.execute()
        rospy.loginfo("[%s] Servoing state machine completed" % rospy.get_name())

    def build_full_sm(self, viz_servo, tag_goal_xyt, find_tag_timeout=None):
        """" Compose the full Smach StateMachine from collected states for PR2 Servoing. """
        sm_pr2_servoing = smach.StateMachine(outcomes=OUTCOMES_SPA)
        with sm_pr2_servoing:
            smach.StateMachine.add('SEARCHING_FOR_TAG',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.BEGIN_FIND_TAG)),
                                   transitions={'succeeded' : 'FIND_TAG'})

            smach.StateMachine.add("FIND_TAG",
                                    FindARTagState(viz_servo, find_tag_timeout),
                                    transitions={"found_tag":"FOUND_TAG",
                                                 "timeout":"TIMEOUT_FIND_TAG",
                                                 "preempted":"preempted",
                                                 "aborted":"aborted"})

            smach.StateMachine.add('FOUND_TAG',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.FOUND_TAG)),
                                   transitions={'succeeded' : 'UI_SERVO_WAIT'})

            smach.StateMachine.add('TIMEOUT_FIND_TAG',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.TIMEOUT_FIND_TAG)),
                                   transitions={'succeeded' : 'FIND_TAG'})

            smach.StateMachine.add('UI_SERVO_WAIT',
                                   BoolTopicState("/pr2_ar_servo/tag_confirm"),
                                   transitions={'true' : 'BEGIN_SERVO',
                                                'false' : 'UI_SERVO_WAIT'})

            smach.StateMachine.add('BEGIN_SERVO',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.BEGIN_SERVO)),
                                   transitions={'succeeded' : 'CC_SERVOING'})

            smach.StateMachine.add('CC_SERVOING',
                                   self.build_cc_servoing(viz_servo, tag_goal_xyt),
                                   transitions={'arm_collision' : 'ARM_COLLISION',
                                                'laser_collision' : 'LASER_COLLISION',
                                                'user_preempted' : 'USER_PREEMPT',
                                                'lost_tag' : 'LOST_TAG',
                                                'succeeded' : 'SUCCESS_SERVO'})

            smach.StateMachine.add('SUCCESS_SERVO',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.SUCCESS_SERVO)),
                                   transitions={'succeeded' : 'succeeded'})

            smach.StateMachine.add('ARM_COLLISION',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.ARM_COLLISION)),
                                   transitions={'succeeded' : 'UI_SERVO_WAIT'})

            smach.StateMachine.add('LASER_COLLISION',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.LASER_COLLISION)),
                                   transitions={'succeeded' : 'UI_SERVO_WAIT'})

            smach.StateMachine.add('LOST_TAG',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.LOST_TAG)),
                                   transitions={'succeeded' : 'FIND_TAG'})

            smach.StateMachine.add('USER_PREEMPT',
                                   PublishState("/pr2_ar_servo/state_feedback",
                                                Int8, Int8(ServoStates.USER_PREEMPT)),
                                   transitions={'succeeded' : 'UI_SERVO_WAIT'})

        return sm_pr2_servoing

    def build_cc_servoing(self, viz_servo, tag_goal_xyt):
        """ Create a Smach Concurrence Container.
            Fill with states for servoing, detecting collision, and processing user preemption.
        """
        def term_cb(outcome_map):
            return True

        def out_cb(outcome_map):
            if outcome_map['ARM_COLLISION_DETECTION'] == 'collision':
                return 'arm_collision'
            #if outcome_map['LASER_COLLISION_DETECTION'] == 'collision':
            #    return 'laser_collision'
            if outcome_map['USER_PREEMPT_DETECTION'] == 'true':
                return 'user_preempted'
            if outcome_map['USER_PREEMPT_DETECTION'] == 'false':
                return 'aborted'
            return outcome_map['SERVOING']

        cc_servoing = smach.Concurrence(
                                outcomes=OUTCOMES_SPA+
                                ['arm_collision', 'laser_collision', 'lost_tag', 'user_preempted'],
                                input_keys=['initial_ar_pose'],
                                default_outcome='aborted',
                                child_termination_cb=term_cb,
                                outcome_cb=out_cb)

        with cc_servoing:
            smach.Concurrence.add('SERVOING',
                                  ServoARTagState(viz_servo, tag_goal_xyt))

            smach.Concurrence.add('ARM_COLLISION_DETECTION',
                                  ArmCollisionDetection())

            #smach.Concurrence.add('LASER_COLLISION_DETECTION',
            #                      LaserCollisionDetection())

            smach.Concurrence.add('USER_PREEMPT_DETECTION',
                                  BoolTopicState("/pr2_ar_servo/preempt"))
        return cc_servoing

class ServoSMManager(object):
    def __init__(self, find_tag_timeout=None):
        self.find_tag_timeout = None
        self.servo_sm_thread = None
        self.tfl = tf.TransformListener()
        self.tag_goal_sub = rospy.Subscriber("ar_servo_goal_data", ARServoGoalData, self.goal_data_cb)
        self.debug_pub = rospy.Publisher("ar_tag_goal", PoseStamped, latch=True, queue_size=1)

    def pose_to_2d(self, ps):
        """ Extract X, Y, Theta from a pose stamped msg."""
        q = (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w)
        rot = tft.euler_from_quaternion(q)[2]
        return ps.pose.position.x, ps.pose.position.y, rot

    def goal_data_cb(self, msg):
        # Receive: desired (goal) pose of tag
        print "Tag goal pose (received): ", msg.tag_goal_pose
        if not 'base_link' in msg.tag_goal_pose.header.frame_id:
            #Transform to base link if necessary
            try:
                now = rospy.get_rostime()
                msg.tag_goal_pose.header.stamp = now
                self.tfl.waitForTransform("base_link", msg.tag_goal_pose.header.frame_id,
                                          now, rospy.Duration(5.0))
                msg.tag_goal_pose = self.tfl.transformPose('base_link', msg.tag_goal_pose)
            except Exception as e:
                rospy.logerr("[%s] TF Error:\r\n%s" % (rospy.get_name(), e.message))
                return
        self.debug_pub.publish(msg.tag_goal_pose)
        tag_goal_xyt = self.pose_to_2d(msg.tag_goal_pose) #convert tag goal to (x,y,theta) triple
        print "Tag goal pose (transformed): ", msg.tag_goal_pose
        print "Tag goal tuple: ", tag_goal_xyt

        #Stop existing state machine if necessary
        if self.servo_sm_thread is not None:
            self.servo_sm_thread.preempt()
            self.servo_sm_thread.join(10)
            if self.servo_sm_thread.is_alive():
                rospy.logerr("[%s] Old servoing thread is not dead yet!" % rospy.get_name())
                return

        #Start new state machine with new data
        self.servo_sm_thread = ServoOnTagGoal(msg.marker_topic,
                                              tag_goal_xyt,
                                              msg.tag_id,
                                              self.find_tag_timeout)
        self.servo_sm_thread.start()
        rospy.loginfo("[%s] Started new servo state machine." % rospy.get_name())


def main():
    rospy.init_node("sm_pr2_servoing")
    find_tag_timeout = None
    servo_manager = ServoSMManager(find_tag_timeout)
    rospy.spin()

if __name__ == '__main__':
    main()
