#!/usr/bin/env python2

import numpy as np
import rospy
import math
import message_filters
from object_fusion_msgs.msg import Detected_Object, Object_List
from visualization_msgs.msg import Marker, MarkerArray

def callback(data,args):
    publisher , color, ns = args
    marker_array = MarkerArray()
    for obj in data.obj_list:

        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.id = obj.obj_id
        marker.ns = ns
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = obj.geometric.x
        marker.pose.position.y = obj.geometric.y
        marker.pose.position.z = 0
        # if ns == "camera":
        #     print(ns,marker.pose.position)
        marker.scale.y = obj.dimension.width;
        marker.scale.x = obj.dimension.length;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.lifetime = rospy.Duration(0.1)

        marker_array.markers.append(marker)
        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.id = obj.obj_id+1000
        marker.ns = ns
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = obj.geometric.x
        marker.pose.position.y = obj.geometric.y
        marker.pose.position.z = 1
        marker.text = str(obj.obj_id)
        marker.scale.y = obj.dimension.width;
        marker.scale.x = obj.dimension.length;
        marker.scale.z = 2;
        marker.color.a = 1.0;
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.lifetime = rospy.Duration(1)
        marker_array.markers.append(marker)
    
    publisher.publish(marker_array)
    pass




def main():
    rospy.init_node("input_vis")
    publisher = rospy.Publisher("/fusion/input/marker", MarkerArray,queue_size = 10)
    rospy.Subscriber("/fusion/input/sensors/camera",Object_List,callback,(publisher,(0,0,255),"camera"))
    rospy.Subscriber("/fusion/input/sensors/radar",Object_List,callback,(publisher,(255,0,0),"radar"))

    rospy.spin()
    pass



if __name__ == '__main__':
    main()