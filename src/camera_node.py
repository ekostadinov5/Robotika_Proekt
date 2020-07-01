#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# Global variables
bridge = CvBridge()
cap = cv2.VideoCapture(0)  # 0 == Laptop's camera


def camera_node():
    global bridge
    global cap

    rospy.init_node('camera_node')

    pub = rospy.Publisher('image', Image, queue_size=5)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            pub.publish(img_msg)

        rate.sleep()


if __name__ == "__main__":
    camera_node()
