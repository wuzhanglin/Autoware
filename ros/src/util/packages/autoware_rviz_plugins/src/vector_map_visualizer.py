#!/usr/bin/env python
import sys
import ast
import math
import random
import rospy
from std_msgs.msg import String
from vector_map_msgs.msg import PointArray
from vector_map_msgs.msg import DTLaneArray
from vector_map_msgs.msg import LaneArray
from vector_map_msgs.msg import LineArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

#TYPE 0 is points, 1 is dtlanes, 2 is lanes, 3 is lines

class Storage:
    def __init__(self):
        self.arg_dict = dict()
        self.radius = sys.float_info.max
        self.focal_point = list()
        self.visualize_types = set()

        self.points_data = None
        self.dtlanes_data = None
        self.lanes_data = None
        self.lines_data = None

        self.rate = None

def parse_arguments(sys_args):
    ret = dict()
    for arg in sys_args:
        key, value = arg.split("=")
        ret[key] = ast.literal_eval(value)
    return ret

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2 + (p0[2] - p1[2])**2)

def find_point(pid):
    for p in store.points_data:
        if p.pid == pid:
            return p

def find_dtlane(did):
    for dtln in store.dtlanes_data:
        if dtln.did == did:
            return dtln

def make_marker(ns, text, marker_id, pos):
    marker = Marker()
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.id = marker_id
    marker.header.frame_id = "map"
    marker.ns = ns
    marker.text = text
    marker.pose.position.x = pos.ly
    marker.pose.position.y = pos.bx
    marker.pose.position.z = pos.h
    marker.pose.orientation.w = 1.0
    marker.pose.orientation.x = .0
    marker.pose.orientation.y = .0
    marker.pose.orientation.z = .0
    marker.scale.x = .2
    marker.scale.y = .2
    marker.scale.z = .2
    marker.color.a = .6
    marker.color.r = random.uniform(0, 1)
    marker.color.g = random.uniform(0, 1)
    marker.color.b = random.uniform(0, 1)
    if random.uniform(0, 1) < 1 / 3:
        marker.color.r = 1
    elif random.uniform(0, 1) < 2 / 3:
        marker.color.g = 1
    else:
        marker.color.b = 1

    return marker

def point_listener_callback(data): 
    global store

    store.points_data = data.data

    if len(store.visualize_types) > 1 and 0 not in store.visualize_types: 
        return

    pub = rospy.Publisher('debugging_point_marker_array', MarkerArray, latch=True, queue_size=10)
    markerArray = MarkerArray()
    count = 0
    #MARKERS_MAX = 2000 

    for p in store.points_data:
        test_point = (p.ly, p.bx, p.h)
        if distance(store.focal_point, test_point) < store.radius:
            print("point.pid:", p.pid)
            marker = make_marker(ns="points", text="PID:" + str(p.pid), marker_id=count, pos=p)
            markerArray.markers.append(marker)
            count += 1
            #if count > MARKERS_MAX:
            #    break

    pub.publish(markerArray)
    print("all_points_published")
    
def dtlane_listener_callback(data):
    global store

    while store.points_data is None:
        store.rate.sleep()

    store.dtlanes_data = data.data

    if len(store.visualize_types) > 1 and 1 not in store.visualize_types: 
        return

    pub = rospy.Publisher('debugging_dtlane_marker_array', MarkerArray, latch=True, queue_size=10)
    markerArray = MarkerArray()
    count = 0
    #MARKERS_MAX = 2000

    for dtln in store.dtlanes_data:
        p = find_point(dtln.pid)
        if distance(store.focal_point, (p.ly, p.bx, p.h)) < store.radius:
            print("dtlane.did:", dtln.did)
            marker = make_marker(ns="dtlanes", text="DID:" + str(dtln.did), marker_id=count, pos=p)            
            markerArray.markers.append(marker)
            count += 1
            #if count > MARKERS_MAX:
            #    break

    pub.publish(markerArray)
    print("all_dtlanes_published")

def lane_listener_callback(data):
    global store

    while store.points_data is None:
        store.rate.sleep()
    while store.dtlanes_data is None:
        store.rate.sleep()

    lanes_data = data.data

    pub = rospy.Publisher('debugging_lane_marker_array', MarkerArray, latch=True, queue_size=10)
    markerArray = MarkerArray()
    count = 0
    #MARKERS_MAX = 2000

    for ln in lanes_data:
        dtln = find_dtlane(ln.did)
        p = find_point(dtln.pid)
        if distance(store.focal_point, (p.ly, p.bx, p.h)) < store.radius:
            print("lane.lnid:", ln.lnid)
            marker = make_marker(ns="lanes", text="LnID:" + str(ln.lnid), marker_id=count, pos=p)            
            markerArray.markers.append(marker)
            count += 1
            #if count > MARKERS_MAX:
            #    break

    pub.publish(markerArray) 
    print("all_lanes_published")     

def line_listener_callback(data):
    global store

    while store.points_data is None:
        store.rate.sleep()

    lines_data = data.data

    pub = rospy.Publisher('debugging_line_marker_array', MarkerArray, latch=True, queue_size=10)
    markerArray = MarkerArray()
    count = 0
    #MARKERS_MAX = 2000

    for l in lines_data:
        p = find_point(l.bpid)
        if distance(store.focal_point, (p.ly, p.bx, p.h)) < store.radius:
            print("line.lid:", l.lid)
            marker = make_marker(ns="lines", text="LID:" + str(l.lid), marker_id=count, pos=p)            
            markerArray.markers.append(marker)
            count += 1
            #if count > MARKERS_MAX:
            #    break

    pub.publish(markerArray) 
    print("all_lines_published") 

def vector_map_listener():
    global store
    store = Storage()

    rospy.init_node('vector_map_debugging_visualizer', anonymous=True)
    store.rate = rospy.Rate(3) # 3hz

    store.arg_dict = parse_arguments(sys.argv[1:])

    if "RADIUS" in store.arg_dict:
        store.radius = float(store.arg_dict["RADIUS"])

    if "FOCAL_POINT" in store.arg_dict:
        store.focal_point = store.arg_dict["FOCAL_POINT"]

    if "TYPE" in store.arg_dict:
        store.visualize_types = set(store.arg_dict["TYPE"])

    rospy.Subscriber("vector_map_info/point", PointArray, point_listener_callback)

    if len(store.visualize_types) < 1 or 1 in store.visualize_types or 2 in store.visualize_types: 
        rospy.Subscriber("vector_map_info/dtlane", DTLaneArray, dtlane_listener_callback)

    if len(store.visualize_types) < 1 or 2 in store.visualize_types:  
        rospy.Subscriber("vector_map_info/lane", LaneArray, lane_listener_callback)

    if len(store.visualize_types) < 1 or 3 in store.visualize_types:  
        rospy.Subscriber("vector_map_info/line", LineArray, line_listener_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        vector_map_listener()
    except rospy.ROSInterruptException:
        pass