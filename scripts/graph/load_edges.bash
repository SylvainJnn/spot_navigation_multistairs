#!/bin/bash

#$1 --> waypoints_2. --> `rospack find spot_navigation_multistairs`/config/waypoints/$1.yaml 	

rosparam load `rospack find spot_navigation_multistairs`/config/waypoints/waypoints_2.yaml 	

rosservice call /rosplan_roadmap_server/load_waypoints
