#!/usr/bin/env python

from __future__ import print_function
import rospy 
from std_srvs.srv import Trigger

#call the services, they are all called with Trigger message and it does not need feedback
def call_services(service_name):
    rospy.wait_for_service(service_name)    
    try:
        order = rospy.ServiceProxy(service_name, Trigger)
        response = order()#call the service
        print(response)
        print(service_name, " works")
        return("")
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)
        print(service_name, "error") 


#call the different services to make spot availible for navigation
def initialise_spot():
    rospy.init_node('initialise_spot')
    call_services('/spot/claim')
    call_services('/spot/power_on')
    call_services('/spot/stand')
    print("Initialisation is over")

if __name__ == '__main__':
    initialise_spot()