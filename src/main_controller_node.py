#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from robotika_proekt.srv import DatabaseService
from robotika_proekt.srv import MessageSendingService
from time import sleep


move = False
wait = False

pub = rospy.Publisher("velocity_cmd", Bool, queue_size=5)


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
    global wait

    rospy.wait_for_service("database_service")
    try:
        service = rospy.ServiceProxy("database_service", DatabaseService)

        response = service(address)

        client = response.client

        if client.id != 0:
            wait = True

            send_request_ms(client)
            sleep(10)

            wait = False

    except rospy.ServiceException as e:
        rospy.loginfo("Service access failed: %s" % e)


def sensor_data_callback(sensor_data_msg):
    global move

    if sensor_data_msg.data:
        move = False  # Obstacle in the way
    else:
        move = True  # Path is clear


def address_callback(addr_msg):
    # rospy.loginfo("Address: " + addr_msg.data)

    send_request_db(addr_msg.data)


def main_controller_node():
    global move
    global pub

    rospy.init_node('main_controller_node')

    rospy.Subscriber("sensor_data", Bool, sensor_data_callback)

    rospy.Subscriber("address", String, address_callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if move and not wait:
            pub.publish(Bool(True))
        else:
            pub.publish(Bool(False))

        rate.sleep()


if __name__ == "__main__":
    main_controller_node()
