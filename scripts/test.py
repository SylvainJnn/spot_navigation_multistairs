#! /usr/bin/env python3
#nothing to see here
import rospy
from waypoints_manager import waypoints_manager


if __name__ == '__main__':
    #rospy.init_node('test_node__waypoints_create')
    print("start creating")
    new_wp = waypoints_manager("G")#take off this -> hard coded 
    print("creating")
    new_wp.create_a_new_waypoint_robot_position()

