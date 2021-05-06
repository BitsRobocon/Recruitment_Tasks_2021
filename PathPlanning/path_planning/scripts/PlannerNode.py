#!/usr/bin/env python3

import sys
import rospy

from path_planning.msg import direction, map_detail
from MapClass import Map

class PlannerNode:
    def __init__(self):
        self.direction_publisher = rospy.Publisher("/direction", direction, queue_size=10)
        # This is the publisher which will publish the direction for the bot to move
        # A general format for publishing has been given below

        self.walls_subscriber = rospy.Subscriber("/walls", map_detail, self.wall_callback)
        # This is the subscriber that will listen for the details about the map that the bot will aquire
        # This data will be send to the wall_callback function where it should be handled

        rospy.sleep(5) # a delay of some time to let the setup of the subscriber and publisher be completed

        # Since we know that the first step the bot will take will be down, we can simply do it here
        temp_val = direction() # make an object of the message type of the publisher
        temp_val.direction = 'down' # assign value to the object. Refer the custom direction.msg in the msg directory
        self.direction_publisher.publish(temp_val) # publish the object

    def wall_callback(self, map_detail):
        # this function will be called everytime the map sends data regarding the map on the '/walls' topic
        # you will recieve the data in the form of the map_detail variable which is an object of custom message type map_detail.msg from the msg directory
        print(map_detail)
        pass # Your code goes here. You need to figure out an algorithm to decide on the best direction of movement of the bot based on the data you have.
        # after deciding on the direction, you need to publish it using the publisher created.

if __name__ == '__main__':
    rospy.init_node('planner_node')
    PlannerNode()
    rospy.spin()
