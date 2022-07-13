#!/usr/bin/env python3

import rospy
import rosservice

import tf
import yaml_manager
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.msg import *
from rosplan_knowledge_msgs.srv import *

from diagnostic_msgs.msg import KeyValue
from rosplan_interface_mapping.srv import *
from rosplan_dispatch_msgs.srv import *

from navigation_2d_spot.srv import CreatePath, CreatePathResponse

from sensor_msgs.msg import LaserScan

#delete all unsed import please
import numpy as np



#this class exists to manage the waypoints: add new waypoints while moving 
class waypoints_manager:

    def __init__(self, new_nb, waypoint_file):#init
        rospy.init_node('waypoints_manager_node')
        self.waypoint_counter = 0       #count the number of waypoints added
        self.nb = new_nb
        self.waypoint_file = waypoint_file #waypoints file, when writting in the yaml file, it needs to know which file to write in 
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan ,self.laser_callback)
        self.scan = LaserScan()
        self.main()



    # ------------- callback -------------
    def laser_callback(self, scan_msg):#/scan callback
        self.scan = scan_msg

    # ------------- useful fucntions to get a specific data -------------
    def get_yaw_from_pose(self, pose):#get yaw's robot from pose from tf lsitener
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(pose[1])
        return(yaw)

    def get_index(self, angle):#get the index scan.ranges
        #angle_min = -pi and angle_max = +pi --> f(-pi) = 0; f(pi)=size; f(0)=s/2; f(pi/2)=3/4*size --> f(x)=size/2(x/pi + 1) 
        nb_elements = len(self.scan.ranges) - 1 #number of element in scan.ranges
        scan_index = int((nb_elements/2) * ((angle/(np.pi)) +1)) #cast as int 
        return(scan_index)

    def get_current_robot_pose_from_tf_listener(self):#from main function in replan.py #it take the robot pose form tf listener and return it 
        print("get a poser from tf listener")
        listener = tf.TransformListener()
        listener.waitForTransform('/map', '/base_link',rospy.Time(), rospy.Duration(4.0))
        try:
            pose = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('Fail')
            exit()

    # ------------- waypoint positions in robot frame -------------
    #scan goes from 0 to 896, angle_min=-pi angle_max=+pi-> I assume it is based on trigonometric circle so --> data[0] = pi / data[size/2] = 0pi / data[size*3/4] = pi/2
    def get_waypoint_position_in_front_of(self): #get waypoints position in robot frame
        # size = len(self.scan_ranges.ranges)
        # x = self.scan_ranges.ranges[int(size*3/4)] #x - 0.5 if x positive otherwise x + 0.5
        # waypoint_position = np.array([float(x), 0.0])
        
        waypoint_position = np.array([2.0,0.0])   #x,y object position, we put in at 2 m in front of the robot
        return(waypoint_position) #for the moment just return an object 2m in front of the robot


    def get_waypoint_position_around(self, angle, scan_index): #get waypoints position in robot frame
        #angle is from -pi tp +pi -> -pi is 0 and +pi is len(self.scan_ranges)-1
        r = self.scan[scan_index]

        if(r > 10):#add limite
            r =10

        Xobject = r * np.cos(angle)
        Yobject = r * np.sin(angle)
        #if not working check if these variables are float
        waypoint_position = np.array([Xobject,Yobject], dtype=float)   
        return(waypoint_position) 

    # ------------- waypoint positions in word frame -------------
    def get_waypoint_position(self, waypoint_position_robot):#get waypoint position in world frame
        robot_pose = self.get_current_robot_pose_from_tf_listener()#get robot pose
        waypoint_pose = robot_pose                                 #for after we need the same format as robot_pose from tf.lsitener, but x and y are going to be changed
        
        yaw = self.get_yaw_from_pose(robot_pose)                                             #robot's yaw from pose
        
        #change base

        R = np.array([[np.cos(yaw),-1*(np.sin(yaw))],   
                      [np.sin(yaw),    np.cos(yaw)]])   #transformation matrix
        
        robot_pose_step = np.array([robot_pose[0][0],
                                    robot_pose[0][1]])  #we work with array like that = [x,y]
        
        #new_pose = robot_pose + R*waypoint_position_robot
        new_pose = np.add(robot_pose_step, np.matmul(R,waypoint_position_robot))
        
        waypoint_pose[0][0] = new_pose[0] #x
        waypoint_pose[0][1] = new_pose[1] #y

        return(waypoint_pose)

    # ------------- get pose functions  -------------
    def get_pose_in_font_of(self):
        waypoint_position_robot = self.get_waypoint_position_in_front_of()   #x,y object position, we put in at 2 m in front of the robot
        waypoint_pose = self.get_waypoint_position(waypoint_position_robot)
        return(waypoint_pose)
    
    def get_pose_around(self, angle):#n repre
        scan_index = self.get_index(angle)
        waypoint_position_robot = self.get_waypoint_position_around(angle, scan_index)#get pose of the waypoint inf robot frame
        waypoint_pose = self.get_waypoint_position(waypoint_position_robot)
        return(waypoint_pose)


    # ------------- Service callers -------------
    def create_waypoint(self, pose, pose_name):# setPose function from replan.py #//later do not tkae robot current pose but a position the robot see !
        # New waypoint
        waypoint = AddWaypointRequest()#what is that object ??????????,
        waypoint.id = pose_name
        waypoint.waypoint.pose.position.x = pose[0][0]
        waypoint.waypoint.pose.position.y = pose[0][1]
        waypoint.waypoint.pose.position.z = pose[0][2]
        waypoint.waypoint.pose.orientation.x = pose[1][0]
        waypoint.waypoint.pose.orientation.y = pose[1][1]
        waypoint.waypoint.pose.orientation.z = pose[1][2]
        waypoint.waypoint.pose.orientation.w = pose[1][3]
        waypoint.connecting_distance = 10 # waht is it exactly ? without it is does not connect
        
        print("the name is :", waypoint.id)    
        
        try:
            # Add waypoint using service 
            add_waypoint = rospy.ServiceProxy('/rosplan_roadmap_server/add_waypoint', AddWaypoint)
            print("Adding NEWy waypoint [ %s %s %s ] "%(round(pose[0][0], 2), round(pose[0][1], 2), pose[0][2]))
            resp1 = add_waypoint(waypoint)
            return resp1
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
    
    def delete_waypoint(self, waypoint_name):#dete a waypoint using RemoveWaypointRequest() object and using /rosplan_roadmap_server/"delte somthing I guess" service
        #remove waypoint service :
        # string id
        # ---
        #     

        None

    # ------------- create waypoints ------------- ## CHANGE THAT, it is 3 times the same dunctions with only one line different
    def create_a_new_waypoint_robot_position(self):  #create a new waypoint on robot current position
        rospy.wait_for_service('/rosplan_roadmap_server/add_waypoint')  #wait for the service to be availible before adding a new waypoint
        pose = self.get_current_robot_pose_from_tf_listener()           #get robot pose

        #to handle the name, for the moment let's say the program create newname different from the waypoints.yaml file
        #--> after let's make the program read that file and see which what's the new name
        new_waypoint_name = "wp" + str(self.nb)                         #give the waypoint name 

        self.create_waypoint(pose, new_waypoint_name)             #create the waypoint ins Rosplan
        yaml_manager.add_waypoint_to_yaml_file(pose, new_waypoint_name, self.waypoint_file)#add the waypoint in yaml file
        #load the new waypoint to connect it : load_edges.bash #right now it deleteds the new home because it is not written in the yaml file // call this also when create new home ? // or get rd of home ? 
        self.waypoint_counter = self.waypoint_counter + 1                                      #increment the varile

    def create_a_new_waypoint_in_front_of(self):
        rospy.wait_for_service('/rosplan_roadmap_server/add_waypoint')  #wait for the service to be availible before adding a new waypoint           
        pose = self.get_pose_in_font_of()

        new_waypoint_name = "wp" + str(self.nb)                         #give the waypoint name 
 
        self.create_waypoint(pose, new_waypoint_name)             #create the waypoint in Rosplan
        yaml_manager.add_waypoint_to_yaml_file(pose, new_waypoint_name, self.waypoint_file)#add the waypoint in yaml file
        #load the new waypoint to connect it : load_edges.bash #right now it deleteds the new home because it is not written in the yaml file // call this also when create new home ? // or get rd of home ? 
        self.waypoint_counter = self.waypoint_counter + 1                                      #increment the varile

    def create_waypoints_around(self, angle):
        rospy.wait_for_service('/rosplan_roadmap_server/add_waypoint')  #wait for the service to be availible before adding a new waypoint           
        pose = self.get_pose_around(angle)

        new_waypoint_name = "wp" + str(self.nb)                         #give the waypoint name 
 
        self.create_waypoint(pose, new_waypoint_name)             #create the waypoint ins Rosplan
        yaml_manager.add_waypoint_to_yaml_file(pose, new_waypoint_name, self.waypoint_file)#add the waypoint in yaml file
        #load the new waypoint to connect it : load_edges.bash #right now it deleteds the new home because it is not written in the yaml file // call this also when create new home ? // or get rd of home ? 
        self.waypoint_counter = self.waypoint_counter + 1                                      #increment the varile


    def create_waypoint_stairs(self):
        None

    def main(self):
        print("creating 3 waypoints around")
        print("angle 0")
        self.create_waypoints_around(0)
        self.nb = str(int(self.nb)+1)
        print("angle pi/2")
        self.create_waypoints_around(np.pi/2)
        self.nb = str(int(self.nb)+1)
        print("angle pi")
        self.create_waypoints_around(np.pi)
        self.nb = str(int(self.nb)+1)

if __name__ == '__main__':
    #rospy.init_node('test_node__waypoints_create')
    print("start creating")
    static_path = "/root/catkin_ws/src/spot_navigation_multistairs/config/waypoints_test.yaml" 
    waypoint_file = static_path + "waypoints_test_autonomus" + ".yaml"#"/root/catkin_ws/src/spot_navigation_multistairs/config/waypoints_test.yaml"  OR just: "staticpath always the same" + "waypoints_test" + ".yaml"
    
    #name = sys.argv[0]
    new_wp = waypoints_manager("10", waypoint_file)#take off this -> hard coded #change 
    #print("creating")
    #new_wp.create_a_new_waypoint_robot_position()
