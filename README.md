

This package provides a navigation system using waypoints for Spot.<br> 
The goal is to make Spot navigate thourght waypoints on different maps and maps on mutliple stairs<br> 
<br> 
To work, this package requieres the package **"navigation_2d_spot"**. If you don't have it because I never saw any documentation of her work, I uploaded it on my github : https://github.com/SylvainJnn/navigation_2d_spot<br> 
<br> 
<br> 
**Functionning**<br> 
&emsp;**rosplan.launch** launchs base.launch, waypoints and rviz<br> 
&emsp;Give a path:<br> 
&emsp;&emsp;You first need to run **replan.py** to give gim a home position (starting position)<br> 
&emsp;&emsp;Run **plan.py** to give the waypoints to go.<br> 
&emsp;&emsp;To create a new waypoint, run **waypoints_manager.py** which call waypoints_manager class, this file as a class which create a new waypoint on the robot current pose (it will also save it in yaml file)<br> 
&emsp;&emsp;To load it, call **load_edges.bash**<br> 
&emsp;Change map:<br> 
&emsp;&emsp;Run **change_map.bash** to change the map, it needs 2 parameters: map name and waypoint file name. <br> 
&emsp;&emsp;Parameters just need the name as long as the map is saved in and "spot_navigation_multistairs/maps" and the waypoint file in "spot_navigation_multistairs/config/waypoints"<br> 
&emsp;&emsp;Runinng change map run the other cash script to delete the current graph, load a new map and load new edges<br> 
&emsp;The stairs mode can be change by laucnhing **stairs_manager.launch** --> normal is set to normal mode, something different implies stair mode<br> 
        



