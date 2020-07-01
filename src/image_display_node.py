#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


# Global variables
bridge = CvBridge()


def callback(img_msg):
    global bridge

    image = None
    try:
        image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Frame", image)
    cv2.waitKey(20)


def image_display_node():
    rospy.init_node('image_display_node')

    rospy.Subscriber("image", Image, callback)

    rospy.spin()


if __name__ == "__main__":
    image_display_node()
