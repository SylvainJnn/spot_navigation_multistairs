
C'est le readme la ou quoi 
C'est le turbo package de ROS pour faire bouger Martin<br> 
Tarpin bien là le sang<br> 


This package provides a navigation system using waypoints for Spot.<br> 
The goal is to make Spot navigate thourght waypoints on different maps and maps on mutliple stairs<br> 


Requirement:<br> 
    **navigation_2d_spot** package by Karolina J.<br> 

How:<br> 
&emsp;rosplan.launch launchs base.launch, waypoints and rviz<br> 
&emsp;give a path:<br> 
&emsp;&emsp;You first need to run **replan.py** to give gim a home position (starting position)<br> 
&emsp;&emsp;Run **plan.py** to give the waypoints to go.<br> 
&emsp;&emsp;to create a new waypoint, run **waypoints_manager.py** which call waypoints_manager class, this file as a class which create a new waypoint on the robot current pose (it will also save it in yaml file)<br> 
&emsp;&emsp;to load it, call **load_edges.bash**<br> 
        
