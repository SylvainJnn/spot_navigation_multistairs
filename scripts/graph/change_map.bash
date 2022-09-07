#!/bin/bash

./delete_graph.bash

#to lead a map, take the first argument
./load_map.bash $1

#second argument is waypoints file
./load_edges.bash $2 