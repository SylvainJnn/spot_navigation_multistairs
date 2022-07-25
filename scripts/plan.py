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

# #I am nearly sure this should be deleted
# def create_new_waypoint(pose, name):
#     """Creates new waypoint for ROSplan

#     Args:
#         pose (PoseStamped): pose of waypoint
#         name (string): name of waypoint 

#     Returns:
#         _type_: service response 
#     """
#     waypoint = AddWaypointRequest()
#     waypoint.id = name
#     waypoint.waypoint.pose.position.x = pose[0][0]
#     waypoint.waypoint.pose.position.y = pose[0][1]
#     waypoint.waypoint.pose.position.z = pose[0][2]
#     waypoint.waypoint.pose.orientation.x = pose[1][0]
#     waypoint.waypoint.pose.orientation.y = pose[1][1]
#     waypoint.waypoint.pose.orientation.z = pose[1][2]
#     waypoint.waypoint.pose.orientation.w = pose[1][3]
  
#     try:
#         # Add waypoint using service 
#         add_waypoint = rospy.ServiceProxy('/rosplan_roadmap_server/add_waypoint', AddWaypoint)
#         resp1 = add_waypoint(waypoint)
#         rospy.loginfo("Plan : New waypoint is added")
#         return resp1
#     except rospy.ServiceException as e:
#         rospy.loginfo("Plan : New waypoint could not be added. Service call failed: %s"%e)




# #my new service 

# def send_wp_for_plan(req):
#     create_plan(req)
#     return(StringResponse("done"))

# def wp_plan_server():
#     rospy.init_node('wp_plan_server')
#     my_service = rospy.Service('wp_plan', String, send_wp_for_plan)
#     print("Working")
#     rospy.spin()


if __name__=="__main__":
    rospy.init_node('plan') 
    # listener = tf.TransformListener()
    # listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
    # """"    
    # try:
    #     pose = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     exit()

    # pose[0][0]= 0
    # pose[0][1] = 1   
    # create_new_waypoint(pose, "wp3") #adding waypoint
    # """
    path ="wp11"#= rospy.get_param("path")
    print(path)
    create_plan(path) 
    #path = "wp10"
    print("this is path\n",path)

    #for wp in path:   
        #create_plan(wp)

