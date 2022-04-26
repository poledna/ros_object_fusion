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
from .Tracked_Object import Tracked_Object
from ..ros2python.Objects import Objects
from ..ros2python.ClassificationMass import ClassificationMass

class Global_Track():
    """docstring for Global_Track"""
    def __init__(self):
        super(Global_Track, self).__init__()
        self.tracked_objects = {}
        self.sensor_property = 0
        self.latest_object_id = 0
        self.timestamp = 0 
        self.latest_id = 1 

    
    def add_object(self, fused_object:Objects,fusing_participants,timestamp,new_object = False):
        self.timestamp = timestamp   
        self.new_object = new_object
        if self.__contains__(fused_object.obj_id):
            self.tracked_objects[fused_object.obj_id].add_fused_object(fused_object,fusing_participants,timestamp)
        else:
            self.tracked_objects[fused_object.obj_id] = Fused_Object(fused_object,fused_object.obj_id,fusing_participants,timestamp)

    def create_new_global_object(self,fused_object:Objects,fusing_participants, timestamp ,new_object = False,classification_mass = 0):
        fused_object.fusion_id = self.latest_id
        self.tracked_objects[self.latest_id] = Fused_Object(fused_object,self.latest_id,fusing_participants,timestamp)
        self.tracked_objects[self.latest_id].current_fused_object.classification_mass = ClassificationMass(classification_mass)

        self.latest_id += 1

    def __contains__(self,item_id):
        return item_id in self.tracked_objects.keys()

    def __str__(self):
        string = f"Global Track with {len(self.tracked_objects.keys())} objects: {list(self.tracked_objects.keys())}"
        for tracked_objects in self.tracked_objects:
            string += f"\n Track  {self.tracked_objects[tracked_objects]}: {tracked_objects}"
        return string

    def to_ros_msg(self, ObjectsList, header):
        ros_msg = ObjectsList(header = header)
        
        for tracked_object in self.tracked_objects.values():
            ros_obj_msg = tracked_object.current_fused_object.to_ros_msg()
            ros_obj_msg.sensors_fused = tracked_object.fusing_participants
            ros_msg.obj_list.append(ros_obj_msg)

        return ros_msg

class Fused_Object(object):
    """docstring for Fused_Object"""
    def __init__(self,fused_object,_id,fusing_participants,timestamp):
        super(Fused_Object, self).__init__()
        self.timestamp = timestamp
        self.id = _id 
        self.current_fused_object = fused_object
        self.fusing_participants = fusing_participants
        self.sensors = []
        
    def __str__(self):        
        string = f"Fused Participants: {self.fusing_participants}"
        # string += f"current_fused_object Participants: f{self.fusing_participants}"
        return string

    def __repr__(self):        
        string = f"Fused Objects: Participants: {self.fusing_participants}"
        # string += f"current_fused_object Participants: f{self.fusing_participants}"
        return string

    def add_fused_object(self,fused_objects,fusing_participants:list,timestamp):
        for participants in fusing_participants:
            if not participants in self.fusing_participants:
                self.sensors.append(participants)
        self.timestamp = timestamp
        self.previous_object = self.current_fused_object
        self.current_fused_object = fused_objects
        self.fusing_participants = fusing_participants


    def set_existance_probability_mass_factors(self,sensor_trust):
        """
        Method to set the new existence probability mass factors after fusion.

        :param sensor_trust
        """
        self.current_fused_object.classification_mass.mass_existance    = (self.current_fused_object.prop_persistance * float(sensor_trust) * self.current_fused_object.prop_existence)
        self.current_fused_object.classification_mass.mass_nonexistance = (self.current_fused_object.prop_persistance * float(sensor_trust) * (1 - self.current_fused_object.prop_existence))
        self.current_fused_object.classification_mass.mass_uncertainity = (1 - (float(self.current_fused_object.classification_mass.mass_existance) + float(self.current_fused_object.classification_mass.mass_nonexistance)))

        self.current_fused_object.classification_mass.list_existance_mass_factor = [self.current_fused_object.classification_mass.mass_existance, self.current_fused_object.classification_mass.mass_nonexistance, self.current_fused_object.classification_mass.mass_uncertainity]

    def existance_mass_prediction(self,prediction_weight):
        """
        Method to predict the existence mass factors for existence probability.

        :param prediction_weight: Defined in the fusion configuration.
        """
        global_object_mass_existance = float(self.current_fused_object.classification_mass.mass_existance)
        global_object_mass_nonexistance = float(self.current_fused_object.classification_mass.mass_nonexistance)
        global_object_mass_uncertainity = float(self.current_fused_object.classification_mass.mass_uncertainity)
        prediction_weight = float(prediction_weight)

        self.current_fused_object.classification_mass.global_predicted_mass_existance = ((1 - prediction_weight) * global_object_mass_existance)
        self.current_fused_object.classification_mass.global_predicted_mass_nonexistance = ((1 - prediction_weight) * global_object_mass_nonexistance)
        self.current_fused_object.classification_mass.global_predicted_mass_uncertainity = ((global_object_mass_uncertainity) + (prediction_weight * (global_object_mass_existance + global_object_mass_nonexistance)))
        self.current_fused_object.classification_mass.global_predicted_masslist = [self.current_fused_object.classification_mass.global_predicted_mass_existance , self.current_fused_object.classification_mass.global_predicted_mass_nonexistance , self.current_fused_object.classification_mass.global_predicted_mass_uncertainity]

