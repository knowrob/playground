#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tests')
import rospy
import smach
import smach_ros

import time
from actionlib import *
from actionlib_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *

import json_prolog.prolog

# moveit
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [-1.5,-2.2,-2,-1.5,0,0]
Q2 = [0.2,-2.0,-1.57,0,0,0]
Q3 = [1.5,-2.2,-2,-1.5,0,0]

client = None
robot = None
scene = None
group = None


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a RobotCommander object.  This object is an interface to
        ## the robot as a whole.
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a PlanningSceneInterface object.  This object is an interface
        ## to the world surrounding the robot.
        rospy.loginfo('Instantiate PlanningSceneInterface')
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a MoveGroupCommander object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the left
        ## arm.  This interface can be used to plan and execute motions on the left
        ## arm.
        rospy.loginfo('Instantiate MoveGroupCommander')
        self.group = moveit_commander.MoveGroupCommander("manipulator")


        ## We create this DisplayTrajectory publisher which is used below to publish
        ## trajectories for RVIZ to visualize.
        rospy.loginfo('Create display_trajectory_publisher')
        self.display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory)

    def execute(self, userdata):

        rospy.loginfo('Executing state Init')
        self.group.set_named_target("up")
        self.group.allow_replanning(True)
        self.group.set_goal_tolerance(0.05)
        self.group.set_planner_id('RRTConnectkConfigDefault')

        plan1 = self.group.plan()
        self.group.go(wait=True)

        rospy.sleep(15)
        #self.group.clear_pose_targets()

        return 'done'

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class MoveIt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['goal_pose'])
        
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a RobotCommander object.  This object is an interface to
        ## the robot as a whole.
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a PlanningSceneInterface object.  This object is an interface
        ## to the world surrounding the robot.
        rospy.loginfo('Instantiate PlanningSceneInterface')
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a MoveGroupCommander object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the left
        ## arm.  This interface can be used to plan and execute motions on the left
        ## arm.
        rospy.loginfo('Instantiate MoveGroupCommander')
        self.group = moveit_commander.MoveGroupCommander("manipulator")


        ## We create this DisplayTrajectory publisher which is used below to publish
        ## trajectories for RVIZ to visualize.
        rospy.loginfo('Create display_trajectory_publisher')
        self.display_trajectory_publisher = rospy.Publisher(
                                            '/move_group/display_planned_path',
                                            moveit_msgs.msg.DisplayTrajectory)

        
    def execute(self, userdata):
      
        rospy.loginfo('Executing state MoveIt')

        self.group.set_pose_target(userdata.goal_pose)
        self.group.allow_replanning(True)
        self.group.set_goal_tolerance(0.05)
        self.group.set_planner_id('RRTConnectkConfigDefault')

        
        rospy.loginfo(userdata.goal_pose)
        
        plan1 = self.group.plan()

        self.group.go(wait=True)

        rospy.sleep(15)
        
        #self.group.clear_pose_targets()

        return 'done'

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class ReadNextGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont', 'finished'],
                                   output_keys=['next_pose'])
        self.counter=0
        self.poses = None
                                   
    def execute(self, userdata):

        if self.poses == None:
          
            # read poses of all objects of a given type
            prolog = json_prolog.prolog.Prolog()
            query = prolog.query("owl_individual_of(Obj, knowrob:'Box-Container'), current_object_pose(Obj, [_,_,_,X,_,_,_,Y,_,_,_,Z,_,_,_,_])")

            # store all results in state-internal list
            self.poses = []
            for solution in query.solutions():

                p = geometry_msgs.msg.Pose()
                p.orientation.w = 1.0
                p.position.x = solution['X']
                p.position.y = solution['Y']
                p.position.z = solution['Z']

                self.poses.append(p)
            query.finish()
            
        print(self.poses)

        # return next pose if available
        if len(self.poses) > 0 :
            userdata.next_pose = self.poses.pop(0)
            return 'cont'
        else:
            return 'finished'

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def main():
    rospy.init_node('smach_example_actionlib')


    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    sm.userdata.tgt_pose = geometry_msgs.msg.Pose()
    
    with sm:

        # Initialize robot pose
        smach.StateMachine.add('INIT', Init(),
                               transitions={'done':'READ_NEXT_POSE'})

        # Read next pose from KB
        smach.StateMachine.add('READ_NEXT_POSE', ReadNextGoal(),
                               transitions={'cont':'MOVE_ARM',
                                            'finished':'succeeded'},
                               remapping={'next_pose':'tgt_pose'})

        # Move to pose
        smach.StateMachine.add('MOVE_ARM', MoveIt(),
                               transitions={'done':'READ_NEXT_POSE'},
                               remapping={'goal_pose':'tgt_pose'})

    # start visualizer and introspection server
    sis = smach_ros.IntrospectionServer('smach_tests', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    
    rospy.spin()
    sis.stop()
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown('All done.')
    

if __name__ == '__main__':
    main()

