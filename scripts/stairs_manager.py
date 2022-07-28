#!/usr/bin/env python3

from __future__ import print_function
import rospy 
from std_srvs.srv import Trigger, SetBool




class stairs_mangager:
    def __init__(self, mode):
        rospy.init_node('spot_stairs_manager')
        self.get_stairs_mode(mode)
        print("start")
        self.set_stair_mode()
        print("done ")
    
    def get_stairs_mode(self, mode):#set thje stair mode, if the global parameter is Normal --> False. the service set spot to stair mode if the given argument is True // to normal mode if argument is false
        if(mode == "normal"):
            self.stairs_mode = False
        else:
            self.stairs_mode = True


    #call the services, they are all called with Trigger message and it does not need feedback
    def call_services_Trigger(service_name):
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

    def set_stair_mode(self):
        service_name = "/spot/stair_mode"
        rospy.wait_for_service(service_name)    
        try:
            order = rospy.ServiceProxy(service_name, SetBool)
            response = order(self.stairs_mode)#call the service
            print(response)
            print(service_name, " works")
            return(True)
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
            print(service_name, "error") 

    #call the different services to make spot availible for navigation
    def pass_to_stair_mode(self):
        self.set_stair_mode(True) #/spot/stairs (??)
        None

    def pass_to_normal_mode(self):
        self.set_stair_mode(False) 
        None

    def allign_stair(self):
        #Once spot arrived the waypoint in front of stairs, make it allign (give a speciffic yaw waypoints for stairs ?)
        None



#normally it works
if __name__ == '__main__':
    mode = rospy.get_param("mode") #get stair mode "normal or other"
    print("really over")
    None