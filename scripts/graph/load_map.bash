#!/bin/bash
#this script launch a new map_server node with a different map
rosnode kill /map_server
map="`rospack find spot_navigation_multistairs`/maps/$1.yaml"
roslaunch spot_navigation_multistairs map_server.launch map_file:=$map


