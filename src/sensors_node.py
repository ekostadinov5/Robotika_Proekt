#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud


pub = rospy.Publisher('sensor_data', Bool, queue_size=5)


def get_distances(points):
    # d0 = points[0].x + points[0].y
    # d1 = points[1].x + points[1].y
    d2 = points[2].x + points[2].y
    d3 = points[3].x + points[3].y
    d4 = points[4].x + abs(points[4].y)
    d5 = points[5].x + abs(points[5].y)
    # d6 = points[6].x + abs(points[6].y)
    # d7 = points[7].x + abs(points[7].y)

    # return d0, d1, d2, d3, d4, d5, d6, d7
    return d2, d3, d4, d5


def callback(sensor_data_msg):
    global pub

    d2, d3, d4, d5 = get_distances(sensor_data_msg.points)

    # rospy.loginfo("Sensor 4: " + str(d0))
    # rospy.loginfo("Sensor 5: " + str(d1))
    rospy.loginfo("Sensor 2: " + str(d2))
    rospy.loginfo("Sensor 3: " + str(d3))
    rospy.loginfo("Sensor 4: " + str(d4))
    rospy.loginfo("Sensor 5: " + str(d5))
    # rospy.loginfo("Sensor 2: " + str(d6))
    # rospy.loginfo("Sensor 3: " + str(d7))

    # Different threshold values for different sensors because not
    # all sensors are in perfect condition. We've determined these
    # values experimentally.
    if d2 < 0.55 or d3 < 0.55 or d4 < 0.55 or d5 < 0.625:
        pub.publish(Bool(True))  # Obstacle in the way
    else:
        pub.publish(Bool(False))  # Path is clear


def sensors_node():
    rospy.init_node('sensors_node')

    rospy.Subscriber("/RosAria/sonar", PointCloud, callback)

    rospy.spin()


if __name__ == "__main__":
    sensors_node()
