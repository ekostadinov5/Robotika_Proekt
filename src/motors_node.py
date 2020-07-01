#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


# Global variables
cmd = ""


def callback(cmd_msg):
    global cmd

    cmd = cmd_msg.data


def motors_node():
    global cmd

    rospy.init_node('motors_node')

    pub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=5)

    rospy.Subscriber("velocity_cmd", String, callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rospy.loginfo(cmd)

        if cmd == "Backwards":  # Rotate to the right / left
            linear = Vector3(x=0.0, y=0.0, z=0.0)
            angular = Vector3(x=0.0, y=0.0, z=0.2)
        elif cmd == "Right":  # Rotate to the right
            linear = Vector3(x=0.0, y=0.0, z=0.0)
            angular = Vector3(x=0.0, y=0.0, z=-0.2)
        elif cmd == "Left":  # Rotate to the left
            linear = Vector3(x=0.0, y=0.0, z=0.0)
            angular = Vector3(x=0.0, y=0.0, z=0.2)
        elif cmd == "Forward":  # Move forward
            linear = Vector3(x=0.125, y=0.0, z=0.0)
            angular = Vector3(x=0.0, y=0.0, z=0.0)
        else:  # Stop
            linear = Vector3(x=0.0, y=0.0, z=0.0)
            angular = Vector3(x=0.0, y=0.0, z=0.0)

        twist = Twist(linear=linear, angular=angular)
        pub.publish(twist)

        rate.sleep()


if __name__ == "__main__":
    motors_node()
