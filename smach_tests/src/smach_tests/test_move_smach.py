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

def joint_traj_goal():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0)),
        JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(5.0))]
    return g


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class MoveIt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'],
                                   input_keys=['goal_pose'])
        self.counter = 0

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
        rospy.loginfo(userdata.goal_pose)
        
        plan1 = self.group.plan()

        self.group.go(wait=True)
        self.group.clear_pose_targets()

        self.counter += 1
        
        return 'done'


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont','done'],
                                   output_keys=['next_pose'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1

            target = geometry_msgs.msg.Pose()
            target.orientation.w = 1.0
            target.position.x = 0.1 + (0.1 * self.counter)
            target.position.y = 0.1 + (0.1 * self.counter)
            target.position.z = 0.5

            userdata.next_pose = target
            return 'cont'
        else:
            return 'done'


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def main():
    rospy.init_node('smach_example_actionlib')


    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    sm0.userdata.tgt_pose = geometry_msgs.msg.Pose()

    # Open the container
    with sm0:

        # simple joint state goal
        #smach.StateMachine.add('MOVE_ARM',
                #smach_ros.SimpleActionState('arm_controller/follow_joint_trajectory',FollowJointTrajectoryAction,goal=joint_traj_goal()),
                #{'succeeded':'ITERATE'})

        # Iterator state
        smach.StateMachine.add('ITERATE', Foo(), 
                               transitions={'cont':'MOVE_ARM',
                                                  'done':'succeeded'},
                               remapping={'next_pose':'tgt_pose'})

        # MoveIt goal
        smach.StateMachine.add('MOVE_ARM', MoveIt(),
                               transitions={'done':'ITERATE'},
                               remapping={'goal_pose':'tgt_pose'})
                

    # Execute SMACH plan
    outcome = sm0.execute()

    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    main()

