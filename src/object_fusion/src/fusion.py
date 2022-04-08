#!/usr/bin/env python3

import rospy
import message_filters
from object_fusion_msgs.msg import Object_List, TrafficUpdateMovingObject

from object_fusion_pypkg.Fusion import Fusion
from object_fusion_pypkg.Callback_Handler import Callback_Handler

def main():
    rospy.init_node("object_fusion")

    object_fusion  = Fusion()

    publisher_fused_data    = rospy.Publisher('/fusion/output', Object_List, queue_size=10)

    callback_handler   = Callback_Handler (publisher_fused_data, object_fusion)
    
    camera_subscriber = rospy.Subscriber("/fusion/input/sensors/camera", Object_List, callback_handler.callback)
    radar_subscriber  = rospy.Subscriber("/fusion/input/sensors/radar" , Object_List, callback_handler.callback)    

    ego_data          = rospy.Subscriber("/fusion/input/ego_position", TrafficUpdateMovingObject, callback_handler.callback)

    rospy.spin()

if __name__ == '__main__':
    main()