#!/bin/bash

rosparam load `rospack find spot_navigation_multistairs`/config/waypoints_test.yaml 	

rosservice call /rosplan_roadmap_server/load_waypoints


