#!/usr/bin/env python3

from __future__ import print_function
import rospy 
from std_srvs.srv import Trigger




class stairs_mangager:
    def __init__(self):
        rospy.init_node('spot_stairs_manager')
        None

    #call the services, they are all called with Trigger message and it does not need feedback
    def call_services(service_name):
        rospy.wait_for_service(service_name)    
        try:
            order = rospy.ServiceProxy(service_name, Trigger)
            response = order()#call the service
            print(response)
            print(service_name, " works")
            return(True)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            print(service_name, "error") 


    #call the different services to make spot availible for navigation
    def pass_to_stair_mode():
        call_services("") #/spot/stairs (??)
        None

    def pass_to_normal_mode():
        None

    def allign_stair(self):
        #Once spot arrived the waypoint in front of stairs, make it allign (give a speciffic yaw waypoints for stairs ?)
        None




if __name__ == '__main__':
    None