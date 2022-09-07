#!/bin/bash

#this script launch a new map_server node with a different map
map="`rospack find spot_navigation_multistairs`/maps/$1.yaml"
rosservice call /change_map map
