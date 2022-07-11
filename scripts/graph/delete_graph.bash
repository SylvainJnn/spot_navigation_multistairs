#!/bin/bash


#delete waypoints
rosparam delete /rosplan_demo_waypoints/wp

#delete edges
rosparam delete /rosplan_demo_waypoints/edge

#delete goal tolerance
rosparam delete /rosplan_waypoints_goal_tolerance