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
    
    def __init__(self, new_nb):
        rospy.init_node('waypoints_manager_node')
        self.waypoint_counter = 0       #count the number of waypoints added
        
        self.nb = new_nb
        rospy.wait_for_service('/rosplan_knowledge_base/update_array')
        rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
        rospy.wait_for_service('/rosplan_planner_interface/planning_server')
        rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
        rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
        #rospy.spin()

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

        print("the name is :", waypoint.id)    
        #waypoint.connecting_distance = 10 #?????
        try:
            # Add waypoint using service 
            add_waypoint = rospy.ServiceProxy('/rosplan_roadmap_server/add_waypoint', AddWaypoint)
            print("Adding NEWy waypoint [ %s %s %s ] "%(round(pose[0][0], 2), round(pose[0][1], 2), pose[0][2]))
            resp1 = add_waypoint(waypoint)
            return resp1
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
        
    def create_a_new_waypoint_robot_position(self):  #create a new waypoint on robot current position
        rospy.wait_for_service('/rosplan_roadmap_server/add_waypoint')  #wait for the service to be availible before adding a new waypoint
        pose = self.get_current_robot_pose_from_tf_listener()

        #to handle the name, for the moment let's say the program create newname different from the waypoints.yaml file
        #--> after let's make the program read that file and see which what's the new name
        #new_waypoint_name = "wp" + str(100) + str(self.waypoint_counter)#wp10X
        new_waypoint_name = "wp" + str(self.nb)

        self.create_a_new_waypoint(pose, new_waypoint_name)
        self.waypoint_counter = self.waypoint_counter + 1                                      #increment the varile
        

