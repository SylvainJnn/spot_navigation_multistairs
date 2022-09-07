#!/usr/bin/env python3

#NOT FINISHED
#The idea was to subscribe to topic /map and then create a python script to call "Loadmap" and "SetMap" 

import rospy
import rosservice
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import SetMap, SetMapResponse
from nav_msgs.srv import LoadMap, LoadMapResponse

class change_map:
    def __init__(self, map_file):
        print('HA')
        rospy.init_node('change_map_nodes_using_services')
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.wait_for_message('/map', OccupancyGrid)
        print("map is get i think")
        self.new_initial_pose = PoseWithCovarianceStamped()
        self.new_initial_pose.header.frame_id ="map"
        self.new_initial_pose.pose.pose.position.x = 0
        self.new_initial_pose.pose.pose.position.y = 0
        self.new_initial_pose.pose.pose.position.z = 0

        self.new_initial_pose.pose.pose.orientation.x = 0
        self.new_initial_pose.pose.pose.orientation.y = 0
        self.new_initial_pose.pose.pose.orientation.z = 0
        self.new_initial_pose.pose.pose.orientation.w = 1
        self.service_name_set_map = "set_map"
        self.new_map = SetMap()
        

        #if we also make the change map not in load_map
        self.map_file = "`rospack find spot_navigation_multistairs`/maps/"+ map_file +".yaml"
        self.service_name_change_map = "change_map"
        self.main()

    def map_callback(self, map_msg):
        self.map = map_msg
        print(self.map.header)
        #proably going to have an issue there


    def change_map(self):#call LoadMap service with the map_url
        rospy.wait_for_service(self.service_name_change_map)
        try:
            load_map = rospy.ServiceProxy(self.service_name_change_map, LoadMap)
            resp = load_map(self.map_file)
            rospy.loginfo("Map has been cahnged")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            print("Service call failed: %s"%e)


    def set_initial_pose(self):
        self.initial_pose = PoseWithCovarianceStamped()
        None

    def write_new_map(self):#create a new map object, inistialise this map the same as the current map
        self.new_map = SetMap()
        self.new_map.map = self.map
        self.new_map.initial_pose = self.initial_pose

    def change_init_pose(self):#call set_map to change robot init pose on new map #need to do more stuff with thr two other function above 
        rospy.wait_for_service(self.service_name_set_map)
        try:
            set_new_map = rospy.ServiceProxy(self.service_name_set_map, SetMap)
            resp = set_new_map(self.map, self.new_initial_pose)
            rospy.loginfo("Map and pose has been cahnged")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            print("Service call failed: %s"%e)

    def main(self):
        #self.change_map()       #call change_map
        #self.write_new_map()
        self.change_init_pose() #call set_map


if __name__ == '__main__':
    change_map("map")
