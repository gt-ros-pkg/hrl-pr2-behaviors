#! /usr/bin/env python
import rospy
import copy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, PlanningScene
import actionlib
from actionlib_msgs.msg import *
import tf
from tf import transformations as tft
from grab_tool_action.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *
from hrl_haptic_manipulation_in_clutter_srvs.srv import *
from hrl_haptic_manipulation_in_clutter_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Bool
from moveit_msgs.msg import PositionIKRequest, RobotState, RobotTrajectory 
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import hrl_msgs.msg
import numpy as np
from math import cos, sin, pi, acos

class toolPickingServer(): #object):
    _feedback=GrabToolFeedback()
    _result=GrabToolResult()

    def __init__(self):
        self.rate=5
        self.timeout=rospy.Duration(30)
        #Arm should be 'r' for right arm or 'l' for left arm
        #Turn on MoveIt and groups for arms and grippers
        robot=moveit_commander.RobotCommander() #interface to robot
        self.scene=moveit_commander.PlanningSceneInterface() #interface to world around the robot
        self.group_rightarm=moveit_commander.MoveGroupCommander('right_arm') #interface to joints of the right arm             
        self.group_leftarm=moveit_commander.MoveGroupCommander('left_arm') #interface to joints of the left arm          
        self.group_leftarm.set_planner_id("RRTConnectkConfigDefault")
        self.group_rightarm.set_planner_id("RRTConnectkConfigDefault")
        
        #Service for IK through moveit
        self.arm_ik=rospy.ServiceProxy('compute_ik', GetPositionIK)       

        #Start action clients for grippers
        self.r_close_gripper_client=actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.l_close_gripper_client=actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
              
        #Services and publishers for Haptic MPC
        self.l_enable_haptic=rospy.ServiceProxy('left_arm/haptic_mpc/enable_mpc', EnableHapticMPC)
        self.r_enable_haptic=rospy.ServiceProxy('right_arm/haptic_mpc/enable_mpc', EnableHapticMPC)
        self.l_goal_pub=rospy.Publisher('left_arm/haptic_mpc/goal_pose', PoseStamped)
        self.r_goal_pub=rospy.Publisher('right_arm/haptic_mpc/goal_pose', PoseStamped)
        self.l_goal_posture_pub = rospy.Publisher("left_arm/haptic_mpc/joint_trajectory", JointTrajectory)
        self.r_goal_posture_pub = rospy.Publisher("right_arm/haptic_mpc/joint_trajectory", JointTrajectory) 
        self.l_haptic_weights = rospy.Publisher("left_arm/haptic_mpc/weights", HapticMpcWeights)
        self.r_haptic_weights = rospy.Publisher("right_arm/haptic_mpc/weights", HapticMpcWeights)

        #Subscribers for MPC
        rospy.Subscriber('left_arm/haptic_mpc/gripper_pose', PoseStamped, self.lgripperPoseCallback)
        rospy.Subscriber('right_arm/haptic_mpc/gripper_pose', PoseStamped, self.rgripperPoseCallback)
        rospy.Subscriber('joint_states', JointState, self.jointStateCallback)

        #Weights for MPC
        #Orient weight for gripper pose
        self.orientation_weights = HapticMpcWeights()
        self.orientation_weights.position_weight = 1
        self.orientation_weights.orient_weight = 1
        self.orientation_weights.posture_weight = 0
     

        #Posture weight for posture poses
        self.posture_weights = HapticMpcWeights()
        self.posture_weights.position_weight = 0
        self.posture_weights.orient_weight = 0
        self.posture_weights.posture_weight = 1
       
        #Define offsets for pick and place
        self.offset_pick=(-0.375, -0.045, 0.04)
        self.offset_place_right=(-0.4, -0.06, 0.025)
        self.offset_place_left=(-0.4, 0.14, 0.025)

        self.tool_listener=tf.TransformListener()
        self.toolbox={'ar_marker_1': 'scratcher', 'ar_marker_2': 'wiper', 'ar_marker_3': 'shaver', 'ar_marker_4': 'toothbrush'}        

        #Set up action server
        self._as=actionlib.SimpleActionServer('grab_tool', GrabToolAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def lgripperPoseCallback(self, msg):
        self.lgripperPose=msg

    def jointStateCallback(self, msg):
        self.jointState=msg

    def rgripperPoseCallback(self, msg):
        self.rgripperPose=msg
 
    def execute_cb(self, goal):
        goal_topic=goal.tag_topic
        arm=goal.arm
        pick=goal.pick

        markers=['ar_marker_1', 'ar_marker_2', 'ar_marker_3', 'ar_marker_4']
        for marker in markers:        
            #Remove markers from last run
            self.scene. remove_attached_object('base_footprint', marker)
        self.add_tools_to_scene(markers) #Add markers for current run 
        
        if pick:
            rospy.loginfo("Starting action server with Tool: %s, Arm: %s, and Pick: Picking tool", self.toolbox[goal_topic], arm) 
            self._feedback.goal_status="Starting action server with Tool: %s, Arm: %s, and Pick: Picking tool" %(self.toolbox[goal_topic], arm)
          
            self._as.publish_feedback(self._feedback)            
            success=self.move_gripper(True, arm) #open gripper in preparation for moving the arm
            self._feedback.goal_status="Opening gripper"            
            self._as.publish_feedback(self._feedback)    
            rospy.loginfo("Opening gripper")        
            plan=self.plan_approach(arm, goal_topic, success, self.offset_pick)       
            self._feedback.goal_status="Planning path to tool"            
            self._as.publish_feedback(self._feedback)             
            rospy.loginfo("Planning path to tool")
            if plan is False:
                self._feedback.goal_status="Planning Failed. Aborting"           
                self._as.publish_feedback(self._feedback)
                rospy.logerr("Planning Failed. Aborting")
                self._as.set_aborted()
            else:
                self._feedback.goal_status="Planning successful, executing plan"           
                self._as.publish_feedback(self._feedback)
                rospy.loginfo("Planning successful, executing plan")               
                success=self.execute_plan(plan, arm, True)
                if success:
                    rospy.loginfo("Executing plan successful. Approaching tool.")
                    self._feedback.goal_status="Executing plan successful. Approaching tool."          
                    self._as.publish_feedback(self._feedback)
                else:
                    self._feedback.goal_status="Executing plan failed. Aborting"
                    self._as.publish_feedback(self._feedback)
                    rospy.logerr("Executing plan failed. Aborting")
                    self._as.set_aborted()
                plan=self.approach_tool(arm, goal_topic, success)
                if plan is False:
                    self._feedback.goal_status="Approaching tool failed. Aborting"           
                    self._as.publish_feedback(self._feedback)
                    rospy.logerr("Approaching tool failed. Aborting")      
                    self._as.set_aborted()                
                else:
                    self._feedback.goal_status="Approaching tool"           
                    self._as.publish_feedback(self._feedback)
                    rospy.loginfo("Approaching tool")                
                    succeeded=self.execute_plan(plan, arm, True)        
                    if succeeded:           
                        success=self.move_gripper(False, arm) #close the gripper, includes recovery behaviors in recovery()
                        if success:
                            self._feedback.goal_status="Tool Successfully Grasped!"           
                            self._as.publish_feedback(self._feedback)
                            rospy.loginfo("Tool Successfully Grasped!")
                            plan=self.remove_tool(arm, goal_topic, success)
                            if plan is not False:
                                rospy.loginfo("Removing Tool")
                                succeeded=self.execute_plan(plan, arm, True)
                                rospy.loginfo("Tool Succcesfully Picked")
                            else: 
                                rospy.loginfo("Remove Tool Failed")
                        else:
                            self._feedback.goal_status="Tool picking failed. Tool not grasped. Aborting"           
                            self._as.publish_feedback(self._feedback)
                            rospy.logerr("Tool picking failed. Tool not grasped. Aborting")
                            self._as.set_aborted()        
                    else:
                        self._feedback.goal_status="Tool picking approach failed: aborting"           
                        self._as.publish_feedback(self._feedback)
                        rospy.logerr("Tool picking approach failed: aborting")
                        self._as.set_aborted()
        elif not pick:
            self._feedback.goal_status="Starting action server with Tool: %s, Arm: %s, and Pick: Returning tool" %(self.toolbox[goal_topic], arm)
            self._as.publish_feedback(self._feedback.goal_status)
            rospy.loginfo("Starting action server with Tool: %s, Arm: %s, and Pick: Returning tool", self.toolbox[goal_topic], arm)
            if goal_topic is 'ar_marker_1':
                plan=self.plan_approach(arm,'ar_marker_2', True, self.offset_place_left) #assumes that tool is already in hand and tools are in the tool box in the right order (1, 2, 3, 4 from left to right)
            
            elif goal_topic is 'ar_marker_2':
                plan=self.plan_approach(arm, 'ar_marker_1', True, self.offset_place_right)

            elif goal_topic is 'ar_marker_3':
                plan=self.plan_approach(arm, 'ar_marker_2', True, self.offset_place_right)
            
            elif goal_topic is 'ar_marker_4': 
                plan=self.plan_approach(arm, 'ar_marker_3', True, self.offset_place_right)

            else:
                self._feedback.goal_status="Chosen marker not in list of tool markers. Tool placement failed"         
                self._as.publish_feedback(self._feedback)
                rospy.logerr("Chosen marker not in list of tool markers. Tool placement failed")
                self._as.set_aborted()
            
            if plan is False:
                self._feedback.goal_status="Planning Failed. Aborting"         
                self._as.publish_feedback(self._feedback)
                rospy.logerr("Planning Failed. Aborting")
                self._as.set_aborted()
            else:
                self._feedback.goal_status="Planning successful, executing plan"           
                self._as.publish_feedback(self._feedback)
                rospy.loginfo("Planning successful, executing plan")           
                success=self.execute_plan(plan, arm, True)
                if success:
                    rospy.loginfo("Executing plan successful. Approaching tool.")
                    self._feedback.goal_status="Executing plan successful. Approaching tool."          
                    self._as.publish_feedback(self._feedback)
                else:
                    rospy.loginfo("Executing plan failed. Aborting")
                    self._feedback.goal_status="Executing plan failed. Aborting"
                    self._as.publish_feedback(self.feedback)
                    self._as.set_aborted()

                plan=self.approach_tool(arm, goal_topic, success)
                if plan is False:
                    self._feedback.goal_status="Approaching tool failed. Aborting"           
                    self._as.publish_feedback(self._feedback)
                    rospy.logerr("Approaching tool failed. Aborting")      
                    self._as.set_aborted()                
                else:
                    self._feedback.goal_status="Approaching tool"           
                    self._as.publish_feedback(self._feedback)
                    rospy.loginfo("Approaching tool")                
                    succeeded=self.execute_plan(plan, arm, True)        
                    if succeeded:           
                        success=self.move_gripper(True, arm)
                        if success:
                            self._feedback.goal_status="Tool Successfully Replaced"          
                            self._as.publish_feedback(self._feedback)
                            rospy.loginfo("Tool Successfully Replaced")
                            self._as.set_success()
                        else:
                            self._feedback.goal_status="Tool replacement failed. Tool not replaced. Aborting"          
                            self._as.publish_feedback(self._feedback)
                            rospy.logerr("Tool replacement failed. Tool not replaced. Aborting")
                            self._as.set_aborted()        
                    else:
                        #TODO: insert recovery behaviors?
                        self._feedback.goal_status="Tool replacement approach failed: aborting"         
                        self._as.publish_feedback(self._feedback)
                        rospy.logerr("Tool replacement approach failed: aborting")
                        self._as.set_aborted()        
            
    def add_tools_to_scene(self, markers):
        #Add tools to the MoveIt planning scene
        for marker_name in markers:
            try:
                now=rospy.Time(0)
                self.tool_listener.waitForTransform('base_footprint', marker_name, now, rospy.Duration(10)) #look up transform between given tool and the base(assumes that base won't move with respect to tools during pick/place task)
                trans, rot=self.tool_listener.lookupTransform('base_footprint', marker_name, now)
                mesh_pose=PoseStamped()
                mesh_pose.header.stamp=rospy.Time.now()
                mesh_pose.header.frame_id='base_footprint'
                mesh_pose.pose.position.x=trans[0]-0.03
                mesh_pose.pose.position.y=trans[1]-0.02
                mesh_pose.pose.position.z=trans[2]+0.05
                mesh_pose.pose.orientation.x=0  #assumes tools are approximately facing forward with respect to the robot's torso
                mesh_pose.pose.orientation.y=0 
                mesh_pose.pose.orientation.z=0 
                mesh_pose.pose.orientation.w=1            
                self.scene.attach_box('base_footprint', marker_name, mesh_pose, [0.1, 0.03, 0.075], ['r_gripper_r_finger_tip_link','r_gripper_r_finger_link', 'r_gripper_l_finger_tip_link', 'r_gripper_l_finger_link', 'l_gripper_r_finger_tip_link', 'l_gripper_r_finger_link', 'l_gripper_l_finger_tip_link', 'l_gripper_l_finger_link'] ) #[l, w, h] of tools
                rospy.loginfo("Added %s to scene" % marker_name) 
               
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):               
                self._feedback.goal_status="Locating %s failed. Please move robot's head so that the tool tags are in full view of the camera. Then press the 'Start' button to try again." % self.toolbox[marker_name]         
                self._as.publish_feedback(self._feedback)
                rospy.logerr("Locating %s failed. Please move robot's head so that the tool tags are in full view of the camera. Then press the 'Start' button to try again.", self.toolbox[marker_name])
                self._as.set_aborted()     
               
    def plan_approach(self, arm, goal_topic, succeed_gripper, offset):
        g=PoseStamped()
        ox, oy, oz=offset
        if arm is 'r'and succeed_gripper:
            self.group_rightarm.set_start_state_to_current_state()
            self.group_rightarm.clear_pose_targets()
            ee_link=self.group_rightarm.get_end_effector_link()
            current_pose=self.group_rightarm.get_current_pose() 
            print (ee_link)        
        elif arm is 'l' and succeed_gripper:
            self.group_leftarm.set_start_state_to_current_state()
            self.group_leftarm.clear_pose_targets()
            ee_link=self.group_leftarm.get_end_effector_link()
            current_pose=self.group_leftarm.get_current_pose() 
            print (ee_link)
    
        if succeed_gripper and not self._as.is_preempt_requested():
            try:
                now=rospy.Time(0)
                self.tool_listener.waitForTransform('base_footprint', goal_topic, now, rospy.Duration(10)) #look up transform between given tool and the base_footprint
                trans, rot=self.tool_listener.lookupTransform('base_footprint', goal_topic, now)
                g.header.stamp=rospy.Time.now()
                g.header.frame_id='base_footprint'
                g.pose.position.x =trans[0]+ox 
                g.pose.position.y =trans[1]+oy
                g.pose.position.z=trans[2]+oz
                q_fin=self.check_angle(current_pose)
                g.pose.orientation.x =q_fin[0] #should be [0, 0, 0, 1] or [1, 0, 0, 0] when base is pointed at tools
                g.pose.orientation.y =q_fin[1]
                g.pose.orientation.z =q_fin[2]
                g.pose.orientation.w =q_fin[3] 
                                    
                #print(trans[0])
                #print(trans[1])
                #print(trans[2])
                #print(q_fin[0])
                #print(q_fin[1])
                #print(q_fin[2])
                #print(q_fin[3])
                
   
                if arm is 'r':
                #Adjust planning parameters for highest likelihood of finding a plan                
                    self.group_rightarm.set_goal_tolerance(0.017)
                    self.group_rightarm.allow_replanning(True)
                    self.group_rightarm.set_pose_reference_frame('base_footprint')
                    self.group_rightarm.set_planning_time(10)
                    self.group_rightarm.set_pose_target(g.pose, ee_link)
                    plan1=self.group_rightarm.plan()
                    #create and publish RobotTrajectory plan
                    if len(plan1.joint_trajectory.points)<1: #check if planning failed
                        return False
                    else:
                        return plan1
                elif arm is 'l':
                    self.group_leftarm.set_goal_tolerance(0.017)
                    self.group_leftarm.allow_replanning(True)
                    self.group_leftarm.set_pose_reference_frame('base_footprint')
                    self.group_leftarm.set_planning_time(10)
                    self.group_leftarm.set_pose_target(g.pose, ee_link)
                    plan1=self.group_leftarm.plan()
                    #create and publish RobotTrajectory plan
                    if len(plan1.joint_trajectory.points)<1: #check if planning failed
                        return False
                    else:
                        return plan1
            
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):               
                return False 
        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()        
        elif not succeed_gripper:
            return False
        else:
            return False
    
    def check_angle(self, current):
        q0=np.array([current.pose.orientation.x,current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w])
        q1=np.array([0.0, 0.0, 0.0, 1.0])
        q2=np.array([1.0, 0.0, 0.0, 0.0])    
        th1=acos((2*(np.dot(q0,q1))**2)-1) #formula for angle of rotation (in radians) between to quaternions. Handles equivalent quaternions (will return 0.0 if q0=-q1 for example
        th2=acos((2*(np.dot(q0,q2))**2)-1)
        if th1>th2:
            return q2
        elif th2>th1:
            return q1

    def execute_plan(self, plan, arm, nompc=True):
        if plan is not False and not self._as.is_preempt_requested()and not nompc:
            last_time=rospy.Time.now()+self.timeout
            goal=JointTrajectory()            
            if arm is 'r':
                self.r_haptic_weights.publish(self.posture_weights)
                goal.header.stamp=rospy.Time.now()
                goal.joint_names=plan.joint_trajectory.joint_names
                goal.points=plan.joint_trajectory.points
                self.r_goal_posture_pub.publish(goal)
                postureErr=np.linalg.norm(np.subtract(plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions, self.rjointState))                  
                while not postureErr<0.08 and not self._as.is_preempt_requested():
                    postureErr=np.linalg.norm(np.subtract(plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions,self.rjointState))
                    if rospy.Time.now()>last_time:
                        self._feedback.goal_status='Timeout exceeded'
                        self._as.publish_feedback(self._feedback)
                        return False
                    elif rospy.Time.now()<last_time:
                        continue
                    rospy.sleep(self.rate)
                if postureErr<0.1:
                    return True
                else:
                    return False
            elif arm is 'l':
                self.l_haptic_weights.publish(self.posture_weights)
                goal.header.stamp=rospy.Time.now()
                goal.joint_names=plan.joint_trajectory.joint_names
                goal.points=plan.joint_trajectory.points
                self.l_goal_posture_pub.publish(goal)
                postureErr=np.linalg.norm(np.subtract(plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions, self.ljointState))
                while not postureErr<0.1 and not self._as.is_preempt_requested():
                    postureErr=np.linalg.norm(np.subtract(plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions,self.ljointState))
                    if rospy.Time.now()>last_time:
                        print('Timeout exceeded')
                        return False
                    elif rospy.Time.now()<last_time:
                        continue
                    rospy.sleep(self.rate)             
                return True
        elif nompc:
            print("Not using MPC")
            if arm is 'r':
                self.group_rightarm.execute(plan)
                #self.r_enable_haptic('True')
                return True
            elif arm is 'l':
                #self.l_enable_haptic('False')
                self.group_leftarm.execute(plan)
                #self.l_enable_haptic('True')
                return True
        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        else:
            return False

    def approach_tool(self, arm, goal_topic, success):  
        if arm is 'r' and success and not self._as.is_preempt_requested():
            self.group_rightarm.set_start_state_to_current_state()
            self.group_rightarm.clear_pose_targets()
            waypoints0=self.group_rightarm.get_current_pose()
            q1=self.check_angle(waypoints0)
            waypoints0.pose.orientation.x=q1[0]
            waypoints0.pose.orientation.y=q1[1]
            waypoints0.pose.orientation.z=q1[2]
            waypoints0.pose.orientation.w=q1[3]            
            waypoints1=waypoints0
            waypoints2=waypoints0
            waypoints3=waypoints0            
            self.group_rightarm.set_pose_reference_frame(waypoints1.header.frame_id)
            
            #create waypoints            
            waypoints1.pose.position.x=waypoints1.pose.position.x+0.05
            waypoints2.pose.position.x=waypoints2.pose.position.x+0.08
            waypoints3.pose.position.x=waypoints3.pose.position.x+0.09 
            eef_step=0.005 #choose a step of 1/2 cm increments
            jump_threshold=0.0 #choose jump threshold of 0.0
            avoid_collisions=False 

            #Compute the cartesian path through the waypoints  
            plan, fraction=self.group_rightarm.compute_cartesian_path([waypoints1.pose, waypoints2.pose, waypoints3.pose], eef_step, jump_threshold, avoid_collisions)
           
            if len(plan.joint_trajectory.points)<1 or fraction<0.5: #check if planning failed or if the fraction of the path followed is below 50%.
                return False                                        #fraction should not be below 98%
            else:
                return plan

        elif arm is 'l' and success and not self._as.is_preempt_requested():
            self.group_leftarm.set_start_state_to_current_state()
            self.group_leftarm.clear_pose_targets()            
            waypoints0=self.group_leftarm.get_current_pose()
            q1=self.check_angle(waypoints0)
            waypoints0.pose.orientation.x=q1[0]
            waypoints0.pose.orientation.y=q1[1]
            waypoints0.pose.orientation.z=q1[2]
            waypoints0.pose.orientation.w=q1[3]            
            waypoints1=waypoints0
            waypoints2=waypoints0
            waypoints3=waypoints0
            self.group_leftarm.set_pose_reference_frame(waypoints1.header.frame_id)            
            
            waypoints1.pose.position.x=waypoints1.pose.position.x+0.05
            waypoints2.pose.position.x=waypoints2.pose.position.x+0.08
            waypoints3.pose.position.x=waypoints3.pose.position.x+0.09 
            eef_step=0.005 #choose a step of 1/2 cm increments
            jump_threshold=0.0 
            avoid_collisions=False   
            plan, fraction=self.group_leftarm.compute_cartesian_path([waypoints1.pose, waypoints2.pose, waypoints3.pose], eef_step, jump_threshold, avoid_collisions )
                 
            if len(plan.joint_trajectory.points)<1 or fraction<0.5:
                return False
            else:
                return plan

        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        else:
            return False

    def remove_tool(self, arm, goal_topic, success): #same as approach tool with different waypoints. Could probably roll them into one function with inputs for waypoints
        if arm is 'r' and success and not self._as.is_preempt_requested():
            self.group_rightarm.set_start_state_to_current_state()
            self.group_rightarm.clear_pose_targets()
            waypoints0=self.group_rightarm.get_current_pose()
            q1=self.check_angle(waypoints0)
            waypoints0.pose.orientation.x=q1[0]
            waypoints0.pose.orientation.y=q1[1]
            waypoints0.pose.orientation.z=q1[2]
            waypoints0.pose.orientation.w=q1[3]            
            waypoints1=waypoints0
            waypoints2=waypoints0
            waypoints3=waypoints0
            waypoints4=waypoints0
            
            self.group_rightarm.set_pose_reference_frame(waypoints1.header.frame_id)
            waypoints1.pose.position.x=waypoints1.pose.position.x-0.05
            waypoints2.pose.position.x=waypoints2.pose.position.x-0.08
            waypoints3.pose.position.x=waypoints3.pose.position.x-0.1 
            eef_step=0.005 #choose a step of 1/2 cm increments
            jump_threshold=0.0 #choose jump threshold of 0.0
            avoid_collisions=False  
            plan, fraction=self.group_rightarm.compute_cartesian_path([waypoints1.pose, waypoints2.pose, waypoints3.pose], eef_step, jump_threshold, avoid_collisions)
                     
            if len(plan.joint_trajectory.points)<1 or fraction<0.5:
                return False
            else:
                return plan
        elif arm is 'l' and success and not self._as.is_preempt_requested():
            self.group_leftarm.set_start_state_to_current_state()
            self.group_leftarm.clear_pose_targets()            
            waypoints0=self.group_leftarm.get_current_pose()
            q1=self.check_angle(waypoints0)
            waypoints0.pose.orientation.x=q1[0]
            waypoints0.pose.orientation.y=q1[1]
            waypoints0.pose.orientation.z=q1[2]
            waypoints0.pose.orientation.w=q1[3]            
            waypoints1=waypoints0
            waypoints2=waypoints0
            waypoints3=waypoints0
            waypoints4=waypoints0
            self.group_leftarm.set_pose_reference_frame(waypoints1.header.frame_id)            
            waypoints1.pose.position.x=waypoints1.pose.position.x-0.05
            waypoints2.pose.position.x=waypoints2.pose.position.x-0.08
            waypoints3.pose.position.x=waypoints3.pose.position.x-0.1 
            eef_step=0.005 #choose a step of 1/2 cm increments
            jump_threshold=0.0 
            avoid_collisions=False   
            plan, fraction=self.group_leftarm.compute_cartesian_path([waypoints1.pose, waypoints2.pose, waypoints3.pose], eef_step, jump_threshold, avoid_collisions )
                       
            if len(plan.joint_trajectory.points)<1 or fraction<0.5:
                return False
            else:
                return plan
        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
        else:
            return False
    

    def move_gripper(self, open_status, arm):
        if open_status and not self._as.is_preempt_requested():  #true is open the gripper
            if arm is 'r':            
                self.r_close_gripper_client.wait_for_server()
                self.r_close_gripper_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.08, max_effort = -1))) #open the gripper with max effort
                self.r_close_gripper_client.wait_for_result()
                result = self.r_close_gripper_client.get_result()
                state=self.r_close_gripper_client.get_state()
            elif arm is 'l':
                self.l_close_gripper_client.wait_for_server()
                self.l_close_gripper_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.08, max_effort = -1))) #open the gripper with max effort
                self.l_close_gripper_client.wait_for_result()
                result = self.l_close_gripper_client.get_result()
                state=self.l_close_gripper_client.get_state()           
            if state == GoalStatus.SUCCEEDED:
                self._feedback.goal_status="Open gripper succeeded" 
                self._as.publish_feedback(self._feedback)               
                return True
            else:
                if result.stalled:
                    self._feedback.goal_status="Obstacle preventing gripper from opening"
                    self._as.publish_feedback(self._feedback)          
                    return False
                else:
                    return False 
        elif self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()         
        else:
            if arm is 'r':   #false is close the gripper
                self.r_close_gripper_client.wait_for_server()
                self.r_close_gripper_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.0, max_effort = 50))) #close the gripper gently
                self.r_close_gripper_client.wait_for_result()
                result = self.r_close_gripper_client.get_result()
                state=self.r_close_gripper_client.get_state()
            elif arm is 'l':
                self.l_close_gripper_client.wait_for_server()
                self.l_close_gripper_client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = 0.0, max_effort = 50))) #close the gripper gently
                self.l_close_gripper_client.wait_for_result()
                result = self.l_close_gripper_client.get_result()
                state=self.l_close_gripper_client.get_state() 
            if state == GoalStatus.SUCCEEDED:
                return False
            else:
                if result.stalled:
                    self._feedback.goal_status="Gripper successfully closed aroud a tool"                   
                    self._as.publish_feedback(self._feedback)
                    print result.position
                    #if result.position>0.048 and result.position<0.057:  #TODO: fix this recovery behavior. Should check if grasping the back of the tool. Could use pressure sensors.
                    #    rospy.loginfo('Tool not correctly grasped. Initiating recovery behavior')                        
                    #    succeed=self.recovery('back', arm)
                    if result.position>0.057: #Checks if grasping the top edge of the tool
                        rospy.loginfo('Tool not correctly grasped. Initiating recovery behavior')    
                        succeed=self.recovery('down', arm)
                    else:
                        succeed=True                    
                    if succeed:                    
                        return True
                    else:
                        return False 
                elif result.reached_goal: 
                    self._feedback.goal_status="Did not grasp tool correctly. Try again"   
                    self._as.publish_feedback(self._feedback)                    
                    return False 
                else:
                    return False

    def recovery(self, command, arm):
        success=self.move_gripper(True, arm) #let go of tool in order to attempt a regrasp
        if arm is 'r' and success:
            self.group_rightarm.set_start_state_to_current_state()
            self.group_rightarm.clear_pose_targets()
            waypoints0=self.group_rightarm.get_current_pose()
            q1=self.check_angle(waypoints0)
        elif arm is 'l' and success:
            self.group_leftarm.set_start_state_to_current_state()
            self.group_leftarm.clear_pose_targets()            
            waypoints0=self.group_leftarm.get_current_pose()
            q1=self.check_angle(waypoints0)
        elif not success:
            return False

        waypoints0.pose.orientation.x=q1[0]
        waypoints0.pose.orientation.y=q1[1]
        waypoints0.pose.orientation.z=q1[2]
        waypoints0.pose.orientation.w=q1[3]            
        waypoints1=waypoints0
        waypoints2=waypoints0
             
        if command is 'back':
            waypoints1.pose.position.x=waypoints1.pose.position.x-0.005
            waypoints2.pose.position.x=waypoints2.pose.position.x-0.01
            command_ready=True
        elif command is 'down':
            waypoints1.pose.position.z=waypoints1.pose.position.z-0.01
            waypoints2.pose.position.z=waypoints2.pose.position.z-0.02
            command_ready=True
        else:
            return False
        eef_step=0.005 #choose a step of 1 cm increments
        jump_threshold=0.0 
        avoid_collisions=False   
        if arm is 'r' and command_ready:
            plan, fraction=self.group_rightarm.compute_cartesian_path([waypoints1.pose, waypoints2.pose], eef_step, jump_threshold, avoid_collisions )
            print fraction            
            if len(plan.joint_trajectory.points)<1 or fraction<0.5:
                return False
            else:
                self.group_rightarm.execute(plan)     
                success=self.move_gripper(False, arm)
                if success:                
                    return True
                else:
                    return False            
        elif arm is 'l'and command_ready:            
            plan, fraction=self.group_leftarm.compute_cartesian_path([waypoints1.pose, waypoints2.pose], eef_step, jump_threshold, avoid_collisions )
            print fraction            
            if len(plan.joint_trajectory.points)<1 or fraction<0.5:
                return False
            else:
                self.group_leftarm.execute(plan)     
                success=sef.move_gripper(False, arm)                
                if success:                
                    return True
                else:
                    return False
        else:
            return False
        
def main():
#if __name__=="__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('grab_tool_server', anonymous=True)
    toolPickingServer()
    while not rospy.is_shutdown():
        rospy.spin()
