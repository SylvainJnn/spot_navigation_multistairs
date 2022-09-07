#!/usr/bin/env python3
# funtions that handle the yaml files caontining waypoints position
import yaml

def open_yaml(file_path):#open a yaml file
    with open(file_path, "r") as stream:
        try:
            waypoint_dictionary = yaml.safe_load(stream)
            return(waypoint_dictionary) #return the yaml file as a dictionary 
        except yaml.YAMLError as exc:
            print(exc)
            return()

def write_in_yaml(file_path, content):#write the string in a file
    f = open(file_path, "w")
    f.write(content)
    f.close()

def add_waypoint_to_yaml_dictionary(waypoint_dictionary, new_wp_name, new_wp_pose): #add a new waypoint to the directory
    #add waypoint to dictionary
    waypoint_dictionary['rosplan_demo_waypoints'].update({new_wp_name: new_wp_pose})            #add the waypoint name and poistion
    waypoint_dictionary['rosplan_waypoints_goal_tolerance'].update({new_wp_name: [0.01, 3.62]}) #add waypoint name and goal tolerance
    return(waypoint_dictionary)
    
def convert_to_yaml(waypoint_dictionary):   #convert the dictionary format to a string // the format for ros to read it
    final_string = ""
    for key in waypoint_dictionary:
        final_string += key + ':'
        for value in waypoint_dictionary[key]:
            final_string += '\n    ' + value + ': ' + str(waypoint_dictionary[key][value])#4 sapces as in the .yaml file 
        final_string += '\n'
    return final_string

def add_waypoint_to_yaml_file(pose, new_wp_name, waypoint_file):#add the waypoint in the yaml file
    #waypoint_file = "/root/catkin_ws/src/spot_navigation_multistairs/config/waypoints_test.yaml" #we want to write in this waypoint // change that with because later we want to work with multiple maps
    new_wp_pose = [ pose[0][0], 
                    pose[0][1], 
                    pose[0][2]]     #x,w,yaw, in fact yaw we can put 0, it does not matter (yet)
    waypoint_dictionary = open_yaml(waypoint_file)
    waypoint_dictionary = add_waypoint_to_yaml_dictionary(waypoint_dictionary, new_wp_name, new_wp_pose)
    waypoint_yaml = convert_to_yaml(waypoint_dictionary)
    write_in_yaml(waypoint_file, waypoint_yaml)





