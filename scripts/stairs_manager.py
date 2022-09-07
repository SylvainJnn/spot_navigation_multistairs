#!/usr/bin/env python3

from __future__ import print_function
import rospy 
from std_srvs.srv import Trigger, SetBool




class stairs_mangager:
    def __init__(self, mode):
        rospy.init_node('spot_stairs_manager')  #init node
        self.get_stairs_mode(mode)              #call function to set the varaible self.stairs_mode to true or calse

        print("start")
        self.set_stair_mode()                   #pass Spot to stair more or to normal mode
        print("done ")
    
    def get_stairs_mode(self, mode):#set the stair mode, if the global parameter is Normal --> False. the service set spot to stair mode if the given argument is True // to normal mode if argument is false
        if(mode == "normal"):
            self.stairs_mode = False
        else:
            self.stairs_mode = True

    #set stair mode by calling the service "/spot/stair_mode"
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
        self.stairs_mode = True
        self.set_stair_mode() 

    def pass_to_normal_mode(self):
        self.stairs_mode = False
        self.set_stair_mode() 

    def allign_stair(self):
        #Once spot arrived the waypoint in front of stairs, make it allign (give a speciffic yaw waypoints for stairs ?)
        None



if __name__ == '__main__':
    mode = rospy.get_param("mode") #get stair mode "normal or other"
    stairs_mangager(mode)
    print("really over")
    None