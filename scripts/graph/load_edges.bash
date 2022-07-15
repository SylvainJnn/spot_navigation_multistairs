#!/bin/bash

#$1 --> waypoints_2. --> `rospack find spot_navigation_multistairs`/config/waypoints/$1.yaml 	
#it looks like it is not getting called ? # called independatly it is ok 
#waypoint_file = $1
waypoint_file="`rospack find spot_navigation_multistairs`/config/waypoints/$1.yaml"
rosparam load $waypoint_file

rosservice call /rosplan_roadmap_server/load_waypoints
