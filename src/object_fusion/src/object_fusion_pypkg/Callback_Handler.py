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

from object_fusion_msgs.msg import Object_List
from object_fusion_msgs.msg import TrafficUpdateMovingObject
from std_msgs.msg import Header

from .ros2python.Sensor_Property import Sensor_Property
from .track.Sensor_Track import Sensor_Track
from .ros2python.Objects import Objects

class Callback_Handler(object):
    def __init__(self, publisher_fused_data, state_and_covariance_fusion):
        super(Callback_Handler, self).__init__()
        self.publisher_fused_data = publisher_fused_data
        self.state_and_covariance_fusion = state_and_covariance_fusion

        self.first_message_from_ego = True
        self.timestamp_previous_message = 0
        self.timestamp_current_message = 0

        self.state_and_covariance_fusion.sensors 

    def sensor_data_parsing(self, sensor_data):
        sensor_property = Sensor_Property()
        sensor_property.from_ros_message(sensor_data.sensor_property)

        for sensor in self.state_and_covariance_fusion.sensors:
            self.state_and_covariance_fusion.sensors[sensor].unset_update_status()

        sensor_id = sensor_property.sensor_id
        if sensor_id in self.state_and_covariance_fusion.sensors.keys():
            self.state_and_covariance_fusion.sensors[sensor_id].set_timestamp(sensor_data.header.stamp.to_sec())
            for sensor_object in sensor_data.obj_list:
                sensor_construct = Objects().from_ros_message(sensor_object)
                self.state_and_covariance_fusion.sensors[sensor_id].add_object(sensor_construct)
                
        else:
            self.state_and_covariance_fusion.sensors[sensor_id] = Sensor_Track(sensor_property)
            self.state_and_covariance_fusion.sensors[sensor_id].set_timestamp(sensor_data.header.stamp.to_sec())
            for sensor_object in sensor_data.obj_list:
                sensor_construct = Objects().from_ros_message(sensor_object)
                self.state_and_covariance_fusion.sensors[sensor_id].add_object(sensor_construct)

        self.state_and_covariance_fusion.sensors[sensor_id].delete_not_updated()

        return sensor_id


    def update_ego_position(self, ego_data):
        self.state_and_covariance_fusion.egoveh.vel.x = ego_data.object.velocity.x
        self.state_and_covariance_fusion.egoveh.vel.y = ego_data.object.velocity.y
        self.state_and_covariance_fusion.egoveh.acc.x = ego_data.object.acceleration.x
        self.state_and_covariance_fusion.egoveh.acc.y = ego_data.object.acceleration.y

        if self.first_message_from_ego:
            self.first_message_from_ego = False
            self.state_and_covariance_fusion.egoveh.neworientation = ego_data.object.orientation.yaw
            self.state_and_covariance_fusion.egoveh.testyaw = self.state_and_covariance_fusion.egoveh.neworientation
            self.state_and_covariance_fusion.egoveh.newyaw = 0
            self.timestamp_previous_message = ego_data.header.stamp.to_sec()
            self.timestamp_current_message = ego_data.header.stamp.to_sec()
        else:
            self.timestamp_previous_message = self.timestamp_current_message
            self.timestamp_current_message = ego_data.header.stamp.to_sec()
            if self.timestamp_previous_message != self.timestamp_current_message:
                self.state_and_covariance_fusion.egoveh.oldorientation = self.state_and_covariance_fusion.egoveh.neworientation
                self.state_and_covariance_fusion.egoveh.neworientation = ego_data.object.orientation.yaw
                self.state_and_covariance_fusion.egoveh.newyaw = self.state_and_covariance_fusion.egoveh.oldorientation - self.state_and_covariance_fusion.egoveh.neworientation

                self.state_and_covariance_fusion.egoveh.t = self.timestamp_current_message - self.timestamp_previous_message
                self.state_and_covariance_fusion.egoveh.yawrate = self.state_and_covariance_fusion.egoveh.newyaw / self.state_and_covariance_fusion.egoveh.t

    def callback(self, ros_msg_data):
        self.state_and_covariance_fusion.time_stamp = rospy.Time.now()
        if isinstance(ros_msg_data,Object_List):
            obj_id = self.sensor_data_parsing(ros_msg_data)

            if self.state_and_covariance_fusion.fuse(obj_id):             
                ros_msg = self.state_and_covariance_fusion.globaltrack.to_ros_msg(Object_List,
                                                                              Header(stamp = rospy.Time().now()))
            
                self.publisher_fused_data.publish(ros_msg)

        
        if isinstance(ros_msg_data,TrafficUpdateMovingObject):
            self.update_ego_position(ros_msg_data)