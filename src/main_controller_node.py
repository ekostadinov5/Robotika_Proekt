#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from robotika_proekt.srv import DatabaseService
from robotika_proekt.srv import MessageSendingService
from time import sleep


def send_request_ms(client):
    rospy.wait_for_service("message_sending_service")
    try:
        service = rospy.ServiceProxy("message_sending_service", MessageSendingService)

        rospy.loginfo("SENDING MESSAGE...")

        response = service(client)

        if response.sent:
            rospy.loginfo("MESSAGE SENT SUCCESSFULLY!")
    except rospy.ServiceException as e:
        rospy.loginfo("Service access failed: %s" % e)


def send_request_db(address):
    rospy.wait_for_service("database_service")
    try:
        service = rospy.ServiceProxy("database_service", DatabaseService)

        response = service(address)

        client = response.client

        if client.id != 0:
            # Stop motors

            rospy.loginfo("STOPPING MOTORS")

            send_request_ms(client)
            sleep(10)

            # Activate motors

            rospy.loginfo("MOTORS ACTIVATED")
    except rospy.ServiceException as e:
        rospy.loginfo("Service access failed: %s" % e)


def callback(addr_msg):
    rospy.loginfo("Address: " + addr_msg.data)

    send_request_db(addr_msg.data)


def main_controller_node():
    rospy.init_node('main_controller_node')

    rospy.Subscriber("address", String, callback)

    rospy.spin()


if __name__ == "__main__":
    main_controller_node()
