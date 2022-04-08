#!/usr/bin/env python
"""
Copyright (C) 2022 Yuri Poledna and Fabio Reway and Redge Castelino and Maikol Drechsler and Shashank Harthi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import numpy as np
import rospy
import math
import message_filters

from object_list.msg import ObjectList, ObjectsList
from visualization_msgs.msg import Marker, MarkerArray

def callback(data,publisher):
    marker_array = MarkerArray()
    for obj in data.obj_list:

        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.id = obj.obj_id+500
        marker.ns = "fusion"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = obj.geometric.x
        marker.pose.position.y = obj.geometric.y
        marker.pose.position.z = 0

        # print(obj.dimension.width,obj.dimension.length)

        marker.scale.x = obj.dimension.length;
        marker.scale.y = obj.dimension.width;
        marker.scale.z = 2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 215/255;
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0.1)
        marker_array.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "vehicle"
        marker.id = obj.obj_id +1000
        marker.ns = "fusion"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = obj.geometric.x
        marker.pose.position.y = obj.geometric.y
        marker.pose.position.z = 1
        marker.text = str(obj.obj_id)
        # print(obj.dimension.width,obj.dimension.length)

        marker.scale.x = obj.dimension.length;
        marker.scale.y = obj.dimension.width;
        marker.scale.z = 2;
        marker.color.a = 1.0;
        marker.color.r = 0.75;
        marker.color.g = 0.5;
        marker.color.b = 0.5
        marker.lifetime = rospy.Duration(0.5)
        marker_array.markers.append(marker)
    
    publisher.publish(marker_array)
    pass


def main():
    rospy.init_node("vis_node")
    publisher = rospy.Publisher("/fusion/output/marker", MarkerArray,queue_size = 10)
    rospy.Subscriber("/fusion/output",ObjectsList,callback,(publisher))
    rospy.spin()
    pass



if __name__ == '__main__':
    main()