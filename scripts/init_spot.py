#!/usr/bin/env python

from __future__ import print_function

import sys 
import rospy 

from std_srvs.srv import Trigger

def call_services(service_name):
    rospy.wait_for_service(service_name)
    try:
        ret = rospy.ServiceProxy(service_name, Trigger)
        print(ret)
        print(service_name, " works")
        return("")
    except:
        print(service_name, "error")


def initialise_spot():
    call_services('/spot/claim')
    call_services('/spot/power_on')
    call_services('/spot/stand')

if __name__ == '__main__':
    initialise_spot()