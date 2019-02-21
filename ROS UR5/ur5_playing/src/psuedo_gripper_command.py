#!/usr/bin/env python

# This script is pretend to be the hololen, which publishes command
# Student: Toan Le - 267945
# Course: Robot Project Work 2018-2019


import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose


def talker():
    pub = rospy.Publisher('gripper_command', Pose, queue_size=10)
    rospy.init_node('psuedo_gripper_command', anonymous=True)
    rate = rospy.Rate(1)  # 0.05hz

    while not rospy.is_shutdown():

        print('=======================')

        command = Pose()

        command.position.x = float(input('Input command: '))


        pub.publish(command)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
