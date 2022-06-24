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

def write_in_yaml(file_path, content):
    f = open(file_path, "w")
    f.write(content)
    f.close()

def add_waypoint_to_yaml_dictionary(waypoint_dictionary, new_wp_name, new_wp_pose):
    #add waypoint to dictionary
    waypoint_dictionary['rosplan_demo_waypoints'].update({new_wp_name: new_wp_pose})
    waypoint_dictionary['rosplan_waypoints_goal_tolerance'].update({new_wp_name: [0.01, 3.62]})
    return(waypoint_dictionary)
    
def convert_to_yaml(waypoint_dictionary):
    final_string = ""
    for key in waypoint_dictionary:
        final_string += key + ':'
        for value in waypoint_dictionary[key]:
            final_string += '\n    ' + value + ': ' + str(waypoint_dictionary[key][value])#4 sapces as in the .yaml file // check if correct later
        final_string += '\n'
    return final_string

def add_waypoint_to_yaml_file(pose, new_wp_name):
    waypoint_file = "/root/catkin_ws/src/navigation_2d_spot/config/waypoints_test.yaml"
    new_wp_pose = pose[pose[0][0], pose[0][1], pose[0][2]]#x,w,yaw, in fact yaw we can put 0, it does not matter (yet)
    waypoint_dictionary = open_yaml(waypoint_file)
    waypoint_dictionary = add_waypoint_to_yaml_dictionary(waypoint_dictionary, new_wp_name, new_wp_pose)
    waypoint_yaml = convert_to_yaml(waypoint_dictionary)
    write_in_yaml(waypoint_file, waypoint_yaml)




