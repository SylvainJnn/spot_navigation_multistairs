#! /usr/bin/env python3

import rospy
from waypoints_manager import waypoints_manager


if __name__ == '__main__':
    #rospy.init_node('test_node__waypoints_create')
    new_wp = waypoints_manager("C")
    print("creating")
    new_wp.create_a_new_waypoint_robot_position()

