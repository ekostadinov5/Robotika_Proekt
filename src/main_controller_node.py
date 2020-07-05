#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from robotika_proekt.srv import DatabaseService
from robotika_proekt.srv import MessageSendingService
from time import sleep


# Global variables
move = ""
wait = False
turning = False


def send_request_ms(client):
    """
    Sends a request to the message sending service node
    """
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
    """
    Sends a request to the database service node and returns its response
    """
    rospy.wait_for_service("database_service")
    try:
        service = rospy.ServiceProxy("database_service", DatabaseService)

        response = service(address)

        return response
    except rospy.ServiceException as e:
        rospy.loginfo("Service access failed: %s" % e)


def address_callback(addr_msg):
    global wait
    global turning

    # rospy.loginfo("Address: " + addr_msg.data)

    # Check if the robot is moving forward or turning
    if not turning:
        # Send a request to the database
        response = send_request_db(addr_msg.data)

        try:
            # Get the client object from the database response
            client = response.client

            # Check if it's a valid client object
            if client.id != 0:
                # Stop the motors
                wait = True

                # Send the client a message
                send_request_ms(client)
                sleep(10)

                # Activate the motors
                wait = False
        except Exception as e:
            pass


def sensor_data_callback(sensor_data_msg):
    global move

    obstacle = sensor_data_msg.data

    if obstacle == "Both":  # If it's a dead end, turn 180 degrees
        move = "Backwards"
    elif obstacle == "Left":  # If there's an obstacle to the left, turn right
        move = "Right"
    elif obstacle == "Right":  # If there's an obstacle to the right, turn left
        move = "Left"
    elif obstacle == "None":  # If there's no obstacle, continue moving forward
        move = "Forward"
    else:  # In any other situation, stop.
        move = "Stop"


def main_controller_node():
    global move
    global wait
    global turning

    # sleep(20)  # For testing

    rospy.init_node('main_controller_node')

    pub = rospy.Publisher("velocity_cmd", String, queue_size=5)

    rospy.Subscriber("sensor_data", String, sensor_data_callback)

    rospy.Subscriber("address", String, address_callback)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if not wait:
            pub.publish(String(move))

            if move == "Backwards":  # Turn 180 degrees
                turning = True
                sleep(16)
                turning = False
            if move == "Right":  # Turn 90 degrees to the right
                turning = True
                sleep(8)
                turning = False
            if move == "Left":  # Turn 90 degrees to the left
                turning = True
                sleep(8)
                turning = False
        else:
            pub.publish(String("Stop"))

        rate.sleep()


if __name__ == "__main__":
    main_controller_node()
