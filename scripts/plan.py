#!/usr/bin/env python3

""" Client class to send all waypoints which should be visited. Method to add waypoints to ROSPlan.

"""

from __future__ import print_function

import sys 
import rospy 
import tf
from navigation_2d_spot.srv import *

from rosplan_knowledge_msgs.msg import *
from rosplan_knowledge_msgs.srv import *

from diagnostic_msgs.msg import KeyValue
from rosplan_interface_mapping.srv import *
from rosplan_dispatch_msgs.srv import *

#from std_msgs.msg import String, StringResponse


#call the service in replan to create the path and make the robot move // need to update that because it does not work with a whole path but only with the last one ???
def create_plan(new_path):
    """Sends plan of waypoints to replan_using_a_new_plan service as a client

    Args:
        new_path (String): All waypoints to be visited

    Returns:
        _type_: service response 
    """
    rospy.wait_for_service('replan_using_a_new_plan')
    try:
        set_plan = rospy.ServiceProxy('replan_using_a_new_plan', CreatePath)
        resp = set_plan(new_path)
        rospy.loginfo("Plan : Plan has been send to the server")
        return new_path
    except rospy.ServiceException as e:
        rospy.loginfo("Plan : New could not be added. Service call failed: %s"%e)
        print("Service call failed: %s"%e)


if __name__=="__main__":
    rospy.init_node('plan') 
    #either hard coded when it rosrun is used OR it take the parameter when it rosplan.launch is used
    path ="wp12"#= rospy.get_param("path") // 
    print(path)
    create_plan(path) 
    #path = "wp10"
    print("this is path\n",path)
