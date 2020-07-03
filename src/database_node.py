#!/usr/bin/env python

import rospy
from robotika_proekt.msg import Client
from robotika_proekt.srv import DatabaseService
from robotika_proekt.srv import DatabaseServiceResponse


# Global variables
db = [
    {
        "id": 1,
        "name": "John",
        "surname": "Smith",
        "address": "adresabr1",
        "to_deliver": True
    },
    {
        "id": 2,
        "name": "Peter",
        "surname": "Johnson",
        "address": "adresabr2",
        "to_deliver": True
    },
    {
        "id": 3,
        "name": "Forest",
        "surname": "Gump",
        "address": "adresabr3",
        "to_deliver": True
    }
]


def find(address):
    global db

    for client in db:
        if client["address"] == address and client["to_deliver"] is True:
            return client
    return None


def handle_request(data):
    client = find(data.address)

    if client is not None:
        rospy.loginfo("CLIENT:")
        rospy.loginfo("\tid: " + str(client["id"]))
        rospy.loginfo("\tname: " + client["name"])
        rospy.loginfo("\tsurname: " + client["surname"])
        rospy.loginfo("\taddress: " + client["address"])
        rospy.loginfo("\tto_deliver: " + str(client["to_deliver"]))

        client["to_deliver"] = False

        client = Client(id=client["id"],
                        name=client["name"],
                        surname=client["surname"],
                        address=client["address"],
                        to_deliver=client["to_deliver"])

        return DatabaseServiceResponse(client)
    return DatabaseServiceResponse(None)


def database_node():
    rospy.init_node('database_node')

    rospy.Service("database_service", DatabaseService, handle_request)

    rospy.spin()


if __name__ == "__main__":
    database_node()
