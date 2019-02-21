#!/usr/bin/env python

# This script is subscriber of hololen command, which publishes command to control UR5's gripper.
# Student: Toan Le - 267945
# Course: Robot Project Work 2018-2019


import rospy
from std_msgs.msg import Bool
from ur_msgs.msg import *
from ur_msgs.srv import SetIO  # In case picking different-size objects, consider SetPayLoad

# Just for testing - could be deleted
from rospy_tutorials.srv import *
from geometry_msgs.msg import Pose


# Handle the gripper
def ur5_gripper(msg):

    print('========== CALLING GRIPPER =============')

    fun = 1  # 1 is index of function which is command to Digital Out service of UR5
    pin = 16  # 16 is index of Gripper port in Digital Out section in UR5. This number also means the Gripper is
    # attached to port 0 of Digital Out
    state = False  # turn off the Gripper

    if msg.position.x == 1:
        print(msg.position.x)
        state = True

    if msg.position.x == 0:
        print(msg.position.x)
        state = False

    print("Grasping state:", state)

    # ========================= TEST ========================== #
    # rospy.wait_for_service('add_two_ints')
    # try:
    #     add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
    #     response = add_two_ints(fun, pin)
    #     print('Result is: ', response.sum)
    # except rospy.ServiceException as e:
    #     print('Service call failed: %s' % e)
    # ======================= TEST END ============================= #

    # ========================= REAL ========================== #
    rospy.wait_for_service('/ur_driver/set_io')
    try:
        grasping = rospy.ServiceProxy('/ur_driver/set_io', SetIO)
        response = grasping(fun, pin, state)
        return response

    except rospy.ServiceException as e:
        print("Gripper calling failed: %s" % e)
    # ========================================================= #


def ur5_gripper_control():
    rospy.init_node('ur5_gripper', anonymous=True)

    # ==================== REAL ================================
    turn_on = rospy.ServiceProxy('/ur_driver/set_io', SetIO)
    on = turn_on(4, 16, 24)

    if on:
        print('=== GRIPPER IS ON ===')
    else:
        print('=== GRIPPER IS OFF ===')
    # ============================================================

    rospy.Subscriber('gripper_command', Pose, ur5_gripper)
    rospy.spin()


if __name__ == '__main__':
    try:
        ur5_gripper_control()

    except rospy.ROSInterruptException:
        print('==== GRIPPER STOPPED ====')
        pass

