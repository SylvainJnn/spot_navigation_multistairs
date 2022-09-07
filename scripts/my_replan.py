#!/usr/bin/env python3

#From Karoline's package, it is the same

""" Service class for planning ROSPlan waypoint path navigation. Equivalent to explore_wp.bash with additional features.

    - Load waypoints into parameter server
    - Add initial location to where spot is 
    - Automatically generate PDDL problem from KB snapshot (e.g. fetch knowledge from KB and create problem.pddl)
    - Make plan
    - Parse plan 
    - Dispatch (execute) plan

"""

import rospy
import rosservice

import tf
import time
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.msg import *
from rosplan_knowledge_msgs.srv import *

from diagnostic_msgs.msg import KeyValue
from rosplan_interface_mapping.srv import *
from rosplan_dispatch_msgs.srv import *

from navigation_2d_spot.srv import CreatePath, CreatePathResponse
import yaml_manager

query = []


# ROSPlan documentation 
"""
When working with Knowledge base, create this
d2 = KnowledgeUpdateServiceArrayRequest()
d2.update_type = [0,]

uint8 ADD_KNOWLEDGE=0
uint8 ADD_GOAL=1
uint8 REMOVE_KNOWLEDGE=2
uint8 REMOVE_GOAL=3
uint8 ADD_METRIC=4
uint8 REMOVE_METRIC=5

http://kcl-planning.github.io/ROSPlan//tutorials/tutorial_08

k.knowledge_type = 1

uint8 INSTANCE = 0
uint8 FACT = 1
uint8 FUNCTION = 2
uint8 EXPRESSION = 3
uint8 INEQUALITY = 4

https://kcl-planning.github.io/ROSPlan//documentation/knowledge/03_KnowledgeItem.html


"""


def setPose(pose, new_wp_name):
    """ Set the current location as a home location 

    Args:
        pose (StampedPose): Current pose of robot in map frame 

    """
    # # Old waypoint 
    # old_waypoint = RemoveWaypointRequest()
    # old_waypoint.id = 'home'
    # # New home waypoint
    waypoint = AddWaypointRequest()
    waypoint.id = new_wp_name
    waypoint.waypoint.pose.position.x = pose[0][0]
    waypoint.waypoint.pose.position.y = pose[0][1]
    waypoint.waypoint.pose.position.z = pose[0][2]
    waypoint.waypoint.pose.orientation.x = pose[1][0]
    waypoint.waypoint.pose.orientation.y = pose[1][1]
    waypoint.waypoint.pose.orientation.z = pose[1][2]
    waypoint.waypoint.pose.orientation.w = pose[1][3]
  
    try:
        # Add waypoint using service 
        add_waypoint = rospy.ServiceProxy('/rosplan_roadmap_server/add_waypoint', AddWaypoint)
        rospy.loginfo("Replan : Adding new home waypoint [ %s %s %s ] "%(round(pose[0][0], 2), round(pose[0][1], 2), pose[0][2]))
        resp1 = add_waypoint(waypoint)
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo("Replan : Adding new home waypoint failed:  %s"%e)


def addInstance():
    """Adding instance, look at domain_spot.pddl 
    """
    an = KnowledgeUpdateServiceArrayRequest()
    an.update_type = [0,]
    a = KnowledgeItem()
    a.knowledge_type = 0
    a.instance_type = 'robot'
    a.instance_name = 'husarion'
    a.attribute_name = ''
    a.function_value = 0.0
    an.knowledge.append(a)
    query.append(an)

def addFact():
    """Adding fact, look at domain_spot.pddl 
    """
    kn = KnowledgeUpdateServiceArrayRequest()
    kn.update_type = [0,]
    k = KnowledgeItem()
    k.knowledge_type = 1
    k.instance_type = ''
    k.instance_name = ''
    k.attribute_name = 'robot_at'
    v1 = KeyValue()
    v1.key = 'r'
    v1.value = 'husarion'
    k.values.append(v1)
    v2 = KeyValue()
    v2.key = 'wp'
    v2.value = 'home'
    k.values.append(v2)
    k.function_value = 0.0
    kn.knowledge.append(k)
    query.append(kn)


def setWaypoint(new_path):
    """Service call to take in a string of all waypoints that should be visited and sending 
       each waypoint to ROSPlan knowledge. These waypoints are being removed from visited instance, 
       so they could get revisited after. 

    Args:
        new_path (String): String of waypoints to be visited
    """
    d = {}
    a = {}
    queryRemove = []
    for word in new_path.split():
        tolerance =  rospy.get_param("rosplan_waypoints_goal_tolerance/" + word)
        print(tolerance)
        d["{0}".format(word)] = KnowledgeUpdateServiceArrayRequest()
        a["{0}".format(word)] = KnowledgeUpdateServiceArrayRequest() # remove visited places
        d["{0}".format(word)].update_type = [1,] # Used to add to the knowledge base
        a["{0}".format(word)].update_type = [2,] # Used to remove from the knowledge base 
        k = KnowledgeItem()
        k.knowledge_type = 1
        k.instance_type = ''
        k.instance_name = ''
        k.attribute_name = 'visited'
        v2 = KeyValue()
        v2.key = 'wp'
        v2.value = word    # to visit place
        k.values.append(v2)
        k.function_value = 0.0
        d["{0}".format(word)].knowledge.append(k)
        a["{0}".format(word)].knowledge.append(k)
        query.append(d["{0}".format(word)])
        queryRemove.append(a["{0}".format(word)])


    try:
        srv = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
        for i in queryRemove:
            resp = srv(i)
        for i in query:
            resp = srv(i)
        rospy.loginfo("Replan : Plan of waypoints was send")
        return resp
    except rospy.ServiceException as e:
        rospy.loginfo("Replan : Sending waypoints to knowledge base failed:  %s"%e)

def genProb():
    """Generate problem 
    """
    try:
        srv = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
        resp = srv()
        return resp
    except rospy.ServiceException as e:
        rospy.loginfo("Replan : Generating problem failed:  %s"%e)

def plan():
    """Planning server
    """
    try:
        srv = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
        resp = srv()
        return resp
    except rospy.ServiceException as e:
        rospy.loginfo("Replan : Planing server failed:  %s"%e)

def parsePlan():
    """Parsing plan
    """
    try:
        srv = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
        resp = srv()
        return resp
    except rospy.ServiceException as e:
        rospy.loginfo("Replan : Parsing plan failed:  %s"%e)

def dispatch():
    """Dispatching plan
    """
    try:
        srv = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
        resp = srv()
        return resp
    except rospy.ServiceException as e:
        rospy.loginfo("Replan : Dispatching plan failed:  %s"%e)


def newPlan(req):
    """Service call to take in waypoints to be visited and sending to the planing interface

    Args:
        req (String): String of waypoints which should be visited
    """
    rospy.loginfo("Replan : Service was called to visit following waypoints: %s " %(req.path))
    addInstance()
    #rospy.sleep(15)
    rospy.loginfo("Replan : Instance added")
    addFact()
    #rospy.sleep(15)
    rospy.loginfo("Replan : Fact added")
    setWaypoint(req.path)
    #rospy.sleep(15)
    rospy.loginfo("Replan : Path was added")
    genProb()
    #rospy.sleep(15)
    rospy.loginfo("Replan : Problem generated")
    plan()
    #rospy.sleep(15)
    rospy.loginfo("Replan : Plan found")
    parsePlan()
    #rospy.sleep(15)
    rospy.loginfo("Replan : Plan parsed")
    dispatch()
    #rospy.sleep(15)
    rospy.loginfo("Replan : Plan dispatched")



def node_init():
    rospy.init_node('replan') 
    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/base_link',rospy.Time(), rospy.Duration(4.0))
    rospy.wait_for_service('/rosplan_roadmap_server/add_waypoint')  
    rospy.wait_for_service('/rosplan_knowledge_base/update_array')
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')

    rospy.loginfo("Replan : All rosplan services accounted for")
    
    # Get the current pose of robot 
    try:
        pose = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('Fail')
        exit()
    # Set this pose as new home waypoint
    name = "home" #for the moment keep this one // the 
    setPose(pose, name)
    #yaml_manager.add_waypoint_to_yaml_file(pose, name, ADD file --> create a rosparam of the current waypoints_file used ? )

    s = rospy.Service('replan_using_a_new_plan', CreatePath, newPlan)
    rospy.spin()#reput the  the spin to call plan.py ? 

"""
either:
    call Karolina's code
    Either we creagte mine and we have to change a few name
        setpose --> set home pose()
        let the rospy.spin()
        

"""

if __name__=="__main__":
    node_init()


