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
import sys

class GlobalObject:
    def __init__(self,obj):
        self.object_id = obj.obj_id
        self.existance_probability = obj.prop_existence
        self.persistance_probability = obj.prop_persistance
        self.mass_existance = None
        self.mass_nonexistance = None
        self.mass_uncertainity = None
        self.list_existance_mass_factor = None
        self.fused_probability_existance = None
        self.fused_probability_nonexistance = None
        self.global_predicted_mass_existance = None
        self.global_predicted_mass_nonexistance = None
        self.global_predicted_mass_uncertainity = None
        self.global_predicted_masslist = None

        self.mass_car = obj.classification_mass[0]
        self.mass_truck = obj.classification_mass[1]
        self.mass_motorcycle = obj.classification_mass[2]
        self.mass_bicycle = obj.classification_mass[3]
        self.mass_pedestrian = obj.classification_mass[4]
        self.mass_stationary = obj.classification_mass[5]
        self.mass_vehicle = obj.classification_mass[6]
        self.mass_vru = obj.classification_mass[7]
        self.mass_traffic = obj.classification_mass[8]
        self.mass_vehicle_stationary = obj.classification_mass[9]
        self.mass_vru_stationary = obj.classification_mass[10]
        self.mass_ignorance = obj.classification_mass[11]
        self.list_classification_mass_factor = obj.classification_mass

    def existance_mass_prediction(self , prediction_weight):
        """
        Method to predict the existence mass factors for existence probability.

        :param prediction_weight: Defined in the fusion configuration.
        """
        global_object_mass_existance = float(self.mass_existance)
        global_object_mass_nonexistance = float(self.mass_nonexistance)
        global_object_mass_uncertainity = float(self.mass_uncertainity)
        prediction_weight = float(prediction_weight)

        self.global_predicted_mass_existance = ((1 - prediction_weight) * global_object_mass_existance)
        self.global_predicted_mass_nonexistance = ((1 - prediction_weight) * global_object_mass_nonexistance)
        self.global_predicted_mass_uncertainity = ((global_object_mass_uncertainity) + (prediction_weight * (global_object_mass_existance + global_object_mass_nonexistance)))
        self.global_predicted_masslist = [self.global_predicted_mass_existance , self.global_predicted_mass_nonexistance , self.global_predicted_mass_uncertainity]


    def set_existance_probability_mass_factors(self, sensor_trust):

        """
        Method to set the new existence probability mass factors after fusion.

        :param sensor:
        """


        self.mass_existance = (self.persistance_probability * float(sensor_trust) * self.existance_probability)
        self.mass_nonexistance = (self.persistance_probability * float(sensor_trust) * (1 - self.existance_probability))
        self.mass_uncertainity = (1 - (float(self.mass_existance) + float(self.mass_nonexistance)))

        self.list_existance_mass_factor = [self.mass_existance, self.mass_nonexistance, self.mass_uncertainity]

