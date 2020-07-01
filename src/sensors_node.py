#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud


# Global variables
pub = rospy.Publisher('sensor_data', String, queue_size=5)


def get_distances(points):
    """
    Calculates the distances to an obstacle for all of the sensors.
    """
    d0 = points[0].x + points[0].y
    d1 = points[1].x + points[1].y
    d2 = points[2].x + points[2].y
    d3 = points[3].x + points[3].y
    d4 = points[4].x + abs(points[4].y)
    d5 = points[5].x + abs(points[5].y)
    d6 = points[6].x + abs(points[6].y)
    d7 = points[7].x + abs(points[7].y)

    return d0, d1, d2, d3, d4, d5, d6, d7


def callback(sensor_data_msg):
    global pub

    d0, d1, d2, d3, d4, d5, d6, d7 = get_distances(sensor_data_msg.points)

    rospy.loginfo("Sensor 0: " + str(d0))
    rospy.loginfo("Sensor 1: " + str(d1))
    rospy.loginfo("Sensor 2: " + str(d2))
    rospy.loginfo("Sensor 3: " + str(d3))
    rospy.loginfo("Sensor 4: " + str(d4))
    rospy.loginfo("Sensor 5: " + str(d5))
    rospy.loginfo("Sensor 6: " + str(d6))
    rospy.loginfo("Sensor 7: " + str(d7))

    if d3 < 0.75 or d4 < 0.75:
        if d0 < 0.75 and d7 < 0.75:  # Obstacles in the front, to the right and to the left - a dead end.
            pub.publish(String("Both"))
        elif d0 < 0.75:  # Obstacles in the front and to the left
            pub.publish(String("Left"))
        elif d7 < 0.75:  # Obstacles in the front and to the right
            pub.publish(String("Right"))
        else:  # Obstacle in the front
            pub.publish("Front")
    else:  # No obstacles
        pub.publish(String("None"))


def sensors_node():
    rospy.init_node('sensors_node')

    rospy.Subscriber("/RosAria/sonar", PointCloud, callback)

    rospy.spin()


if __name__ == "__main__":
    sensors_node()
