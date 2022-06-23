#!/usr/bin/env python3

import yaml

def open_yaml(file):
    with open(file, "r") as stream:
        try:
            wp_file = yaml.safe_load(stream)
            return(wp_file)
        except yaml.YAMLError as exc:
            print(exc)
            return()

#???????????
def write_yaml(file_name, waypoints_dictionary):
    with open(file_name, 'w') as file:
        yaml.dump(waypoints_dictionary, file, default_flow_style=False)


def add_waypoints_to_yaml(waypoints_file, new_wp_name, new_wp_pose):
    #add waypoint to dictionary
    waypoints_file['rosplan_demo_waypoints'].update({new_wp_name: new_wp_pose})
    waypoints_file['rosplan_waypoints_goal_tolerance'].update({new_wp_name: [0.01, 3.62]})
    return(waypoints_file)
    


