#!/usr/bin/env python

import rospy
from robotika_proekt.srv import MessageSendingService
from robotika_proekt.srv import MessageSendingServiceResponse


def handle_request(data):
    client = data.client

    rospy.loginfo("Dear Mr." + client.surname)
    rospy.loginfo("Your packet has arrived.")

    return MessageSendingServiceResponse(True)


def message_sending_node():
    rospy.init_node('message_sending_node')

    rospy.Service("message_sending_service", MessageSendingService, handle_request)

    rospy.spin()


if __name__ == "__main__":
    message_sending_node()
