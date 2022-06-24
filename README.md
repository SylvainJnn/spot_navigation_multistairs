
C'est le readme la ou quoi 
C'est le turbo package de ROS pour faire bouger Martin
Tarpin bien là le sang


This package provides a navigation system using waypoints for Spot.
The goal is make to make this package works for multiple stairs.


Requirement:
    navigation_2d_spot package by Karolina J.

How:
    rosplan.launch launchs base.launch, waypoints and rviz
    give a path:
        You first need to run replan.py to give gim a home position (starting position)
        Run plan.py to give the waypoints to go.
        to create a new waypoint, run test.py which call waypoints_manager.py, this file as a class which create a new waypoint on the robot current pose
