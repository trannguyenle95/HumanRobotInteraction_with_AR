#!/usr/bin/env python

# This script is subscriber of hololen command, which publishes command from hoblolen user.
# The code is written based on Moveit's move_group_Python_interface.py
# Student: Toan Le - 267945
# Course: Robot Project Work 2018-2019


import sys
import rospy
import copy
import moveit_commander
import geometry_msgs.msg


def move_ur5(msg, args):

    robot = args[0]
    ur5 = args[1]

    end_effector = ur5.get_end_effector_link()

    # Allow replanning to increase the odds of a solution
    ur5.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    ur5.set_goal_position_tolerance(0.01)
    ur5.set_goal_orientation_tolerance(0.1)

    print('========= PLANNING ==========')

    desired_pose = msg.position
    print(desired_pose)
    print('========================================================')

    # ==================================== Joint Move Done
    start_point = ur5.get_current_pose(end_effector).pose
    pose_target = copy.deepcopy(start_point)
    pose_target.position.x = desired_pose.x
    pose_target.position.y = desired_pose.y
    pose_target.position.z = desired_pose.z

    # Set the internal state to the current state
    ur5.set_start_state_to_current_state()

    plan = ur5.plan(pose_target)

    print('===== MOVING =====')
    ur5.execute(plan, wait=True)

    print('==== ROBOT STATE AFTER MOVING ====')
    print(robot.get_current_state().joint_state.position)
    print('=============')

    print('==== EE POSE ====')
    print(ur5.get_current_pose().pose)
    print('=============')


def subscriber():
    print()
    print('========================================================')

    # Initializing moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('CartesianPose', anonymous=True)

    # Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()  # WHAT CAN WE DO WITH THIS???

    # Instantiate UR5 as group of joints. This interface can be used to plan and execute motion of UR5
    ur5 = moveit_commander.MoveGroupCommander('manipulator')

    # Print some features of UR5
    print('======= Reference frame:', ur5.get_planning_frame())
    print('======= End Effector:', ur5.get_end_effector_link())
    # Set the reference frame for pose targets
    reference_frame = "/base_link"

    # Set the ur5_arm reference frame accordingly
    ur5.set_pose_reference_frame(reference_frame)

    print('======= ROBOT STATE =======')
    print(robot.get_current_state().joint_state.position)
    print('================')

    print('==== EE POSE ====')
    print(ur5.get_current_pose().pose)
    print('=============')

    rospy.Subscriber('psuedo_hololen', geometry_msgs.msg.Pose, move_ur5, (robot, ur5))

    rospy.spin()


if __name__ == '__main__':
    try:
        subscriber()

    except rospy.ROSInterruptException:
        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        print('==== STOPPING ====')
        pass
