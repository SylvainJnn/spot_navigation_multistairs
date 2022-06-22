#!/usr/bin/env python3

import rospy
import rosservice

import tf
import time
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.msg import *
from rosplan_knowledge_msgs.srv import *

from diagnostic_msgs.msg import KeyValue
from rosplan_interface_mapping.srv import *
from rosplan_dispatch_msgs.srv import *

from navigation_2d_spot.srv import CreatePath, CreatePathResponse


#this class exists to manage the waypoints: add new waypoints while moving 
class waypoints_manager:
    
    def __init__(self):
        rospy.init_node('waypoints_manager')
        self.waypoint_counter = 0       #count the number of waypoints added
        rospy.spin()

    def get_current_robot_pose_from_tf_listener(self):#from main function in replan.py
        print("get a poser from tf listener")
        listener = tf.TransformListener()
        listener.waitForTransform('/map', '/base_link',rospy.Time(), rospy.Duration(4.0))
        
        try:
            pose = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('Fail')
            exit()
        

    def create_a_new_waypoint(self, pose, pose_name):# setPose function from replan.py
        # we do not want to remove anywaypoints // I think just by adding a waypoints with the same name, it removes that waypoint
        # # Old waypoint 
        # old_waypoint = RemoveWaypointRequest()
        # old_waypoint.id = 'home'
        # New home waypoint
        waypoint = AddWaypointRequest()#what is that object ??????????,
        waypoint.id = pose_name
        waypoint.waypoint.pose.position.x = pose[0][0]
        waypoint.waypoint.pose.position.y = pose[0][1]
        waypoint.waypoint.pose.position.z = pose[0][2]
        waypoint.waypoint.pose.orientation.x = pose[1][0]
        waypoint.waypoint.pose.orientation.y = pose[1][1]
        waypoint.waypoint.pose.orientation.z = pose[1][2]
        waypoint.waypoint.pose.orientation.w = pose[1][3]
    
        #waypoint.connecting_distance = 10 #?????
        try:
            # Add waypoint using service 
            add_waypoint = rospy.ServiceProxy('/rosplan_roadmap_server/add_waypoint', AddWaypoint)
            print("Adding new home waypoint [ %s %s %s ] "%(round(pose[0][0], 2), round(pose[0][1], 2), pose[0][2]))
            resp1 = add_waypoint(waypoint)
            return resp1
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
        
    def create_a_new_waypoint_robot_position(self):#create a new waypoint on robot current position
        rospy.wait_for_service('/rosplan_roadmap_server/add_waypoint')  #wait for the service to be availible before adding a new waypoint
        pose = self.get_current_robot_pose_from_tf_listener()

        #to handle the name, for the moment let's say the program create newname different from the waypoints.yaml file
        #--> after let's make the program read that file and see which what's the new name
        new_waypoint_name = "wp10" + str(self.waypoint_counter)

        self.create_a_new_waypoint(pose, new_waypoint_name)

