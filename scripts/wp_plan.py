#!/usr/bin/env python3

""" Client class to send all waypoints which should be visited. Method to add waypoints to ROSPlan.

"""
#not used yet 
from __future__ import print_function

import sys 
import rospy 
import tf

from std_msgs.msg import String, StringResponse



def wp_plan_client(waypoints_to_visit):
    for waypoint in waypoints_to_visit:
        rospy.wait_for_service('wp_plan_server')
        try:
            next_waypoint = rospy.ServiceProxy('wp_plan_server',String)
            response = next_waypoint(waypoint)
            print(response)
            print('wp_plan_server', " works")
            return("done I guess")
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            print('wp_plan_server', "error") 


if __name__=="__main__":
    rospy.init_node('wp_plan') 