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
class Objects(object):
    """docstring for Objects"""
    def __init__(self):
        self.fusion_id = 0
        pass

    def __call__(self):
        self.__init__()
        return self

    def create(self, obj_id, time, geometric, covariance, dimension, prop_existence,prop_nonexistence,prop_persistance,prop_mov,classification,classification_mass,features,sensors_fused):
        super(Objects, self).__init__()
        self.obj_id = obj_id
        self.time = time
        self.geometric = geometric
        self.covariance = covariance
        self.dimension = dimension
        self.prop_existence = prop_existence
        self.prop_nonexistence = prop_nonexistence
        self.prop_persistance = prop_persistance
        self.prop_mov = prop_mov
        self.classification = classification
        self.classification_mass = classification_mass
        self.features = features
        self.sensors_fused = sensors_fused

    def from_ros_message(self, ros_msg_data):
        super(Objects, self).__init__()
        self.obj_id = ros_msg_data.obj_id
        self.time = ros_msg_data.time
        self.geometric = ros_msg_data.geometric
        self.covariance = ros_msg_data.covariance
        self.dimension =ros_msg_data. dimension
        self.prop_existence = ros_msg_data.prop_existence
        self.prop_nonexistence = ros_msg_data.prop_nonexistence
        self.prop_persistance = ros_msg_data.prop_persistance
        self.prop_mov = ros_msg_data.prop_mov
        self.classification = ros_msg_data.classification
        self.classification_mass = ros_msg_data.classification_mass
        self.features = ros_msg_data.features
        self.sensors_fused = ros_msg_data.sensors_fused
        return self
    
    def to_ros_msg(self):
        from object_list.msg import ObjectList
        return ObjectList(self.fusion_id,
            self.time, 
            self.geometric, 
            self.covariance, 
            self.dimension, 
            self.prop_existence, 
            self.prop_nonexistence, 
            self.prop_persistance, 
            self.prop_mov,
            self.classification, 
            self.classification_mass,
            self.features, 
            self.sensors_fused)