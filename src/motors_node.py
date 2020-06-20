#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


cmd = False  # True == move, False == stop


def callback(cmd_msg):
    global cmd
    cmd = cmd_msg.data


def motors_node():
    global cmd

    rospy.init_node('motors_node')

    pub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=5)

    rospy.Subscriber("velocity_cmd", Bool, callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rospy.loginfo("MOVE") if cmd else rospy.loginfo("STOP")

        if cmd:
            linear = Vector3(x=0.05, y=0.0, z=0.0)
            angular = Vector3(x=0.0, y=0.0, z=0.0)

            twist = Twist(linear=linear, angular=angular)

            pub.publish(twist)

        rate.sleep()


if __name__ == "__main__":
    motors_node()
