#!/usr/bin/env python3
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

import rospy
import message_filters
from object_fusion_msgs.msg import Object_List, TrafficUpdateMovingObject

from object_fusion_pypkg.Fusion import Fusion
from object_fusion_pypkg.Callback_Handler import Callback_Handler

def main():
    rospy.init_node("object_fusion")

    object_fusion  = Fusion(mahalanobis_distance_threshold = 15)

    publisher_fused_data    = rospy.Publisher('/fusion/output', Object_List, queue_size=10)

    callback_handler   = Callback_Handler (publisher_fused_data, object_fusion)
    
    camera_subscriber = rospy.Subscriber("/fusion/input/sensors/camera", Object_List, callback_handler.callback)
    radar_subscriber  = rospy.Subscriber("/fusion/input/sensors/radar" , Object_List, callback_handler.callback)    

    ego_data          = rospy.Subscriber("/fusion/input/ego_position", TrafficUpdateMovingObject, callback_handler.callback)

    rospy.spin()

if __name__ == '__main__':
    main()