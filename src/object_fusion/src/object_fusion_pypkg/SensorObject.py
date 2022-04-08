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


class SensorObject:
    def __init__(self,obj,sensor_property):
        self.ids = obj.obj_id
        self.sensor_trust = sensor_property.trust_existance
        self.existance_probability = obj.prop_existence
        self.persistance_probability = obj.prop_persistance
        self.classification_vector = [obj.classification.car, obj.classification.truck, obj.classification.motorcycle, obj.classification.bicycle, obj.classification.pedestrian, obj.classification.stacionary, obj.classification.other]

        if (np.sqrt(np.square(obj.geometric.vx) + np.square(obj.geometric.vy)) > 0  and np.sqrt(np.square(obj.geometric.vx) + np.square(obj.geometric.vy)) < 1):
            self.probability_object_moved = 0.3

        elif np.sqrt(np.square(obj.geometric.vx) + np.square(obj.geometric.vy)) > 1:
            self.probabiliry_object_moved = 0.6
        else:
            self.probabiliry_object_moved = 0.01

        self.feature_vector = None
        self.sensor = sensor_property
        #self.sensor = None

        #self.state_vector_EGOFOR = None
        self.mass_existance = None
        self.mass_nonexistance = None
        self.mass_uncertainity = None
        self.list_existance_mass_factor = None

        self.mass_car = None
        self.mass_truck = None
        self.mass_motorcycle = None
        self.mass_bicycle = None
        self.mass_pedestrian = None
        self.mass_stationary = None
        self.mass_vehicle = None
        self.mass_vru = None
        self.mass_traffic = None
        self.mass_vehicle_stationary = None
        self.mass_vru_stationary = None
        self.mass_ignorance = None
        self.list_classification_mass_factor = None


    def set_existance_probability_mass_factors(self):
        """
        Calculate the exitence probability mass factors for the sensor object using the existence vector.
        """

        self.mass_existance = (float(self.persistance_probability) * float(self.sensor_trust) * float(
            self.existance_probability))
        self.mass_nonexistance = (float(self.persistance_probability) * float(self.sensor_trust) * float(
            (1 - self.existance_probability)))
        self.mass_uncertainity = (1 - (float(self.mass_existance) + float(self.mass_nonexistance)))

        self.list_existance_mass_factor = [self.mass_existance, self.mass_nonexistance, self.mass_uncertainity]

    def set_classification_mass_factors(self):
        """
        Claculate the classification probability mass factors for the sensor object using the classification vector.
        """
        object_probability_car = self.classification_vector[0]
        object_probability_truck = self.classification_vector[1]
        object_probability_motorcycle = self.classification_vector[2]
        object_probability_bicycle = self.classification_vector[3]
        object_probability_pedestrian = self.classification_vector[4]
        object_probability_stationary = self.classification_vector[5]
        object_probability_other = self.classification_vector[6]
        object_moved = 0.5#self.probability_object_moved    #Need to determine

        self.mass_car = float(self.sensor.trust_car) * float(object_probability_car)
        self.mass_truck = float(self.sensor.trust_truck) * float(object_probability_truck)
        self.mass_motorcycle = float(self.sensor.trust_motorcycle) * float(object_probability_motorcycle)
        self.mass_bicycle = float(self.sensor.trust_bicycle) * float(object_probability_bicycle)
        self.mass_pedestrian = float(self.sensor.trust_pedestrian) * float(object_probability_pedestrian)
        self.mass_stationary = float(self.sensor.trust_stationary) * float(object_probability_stationary)

        mass_sum = float(self.mass_car) + float(self.mass_truck) + float(self.mass_motorcycle) + float(self.mass_bicycle) + float(self.mass_pedestrian)
        if mass_sum == 0:
            mass_sum=0.1

        object_probability_vehicle = (float(self.mass_car) + float(self.mass_truck) + float(self.mass_motorcycle)) / (mass_sum)
        object_probability_vru = (float(self.mass_bicycle) + float(self.mass_pedestrian)) / (mass_sum)

        self.mass_vehicle = (float(object_moved)) * (float(object_probability_vehicle)) * (
                    ((1 - float(self.sensor.trust_car)) * float(object_probability_car)) + (
                        (1 - float(self.sensor.trust_truck)) * float(object_probability_truck)) + (
                                (1 - float(self.sensor.trust_motorcycle)) * float(object_probability_motorcycle)))
        self.mass_vru = (float(object_moved)) * (float(object_probability_vru)) * (
                    ((1 - float(self.sensor.trust_bicycle)) * float(object_probability_bicycle)) + (
                        (1 - float(self.sensor.trust_pedestrian)) * float(object_probability_pedestrian)))
        self.mass_traffic = ((float(object_moved)) * (float(object_probability_vru)) * (
                    ((1 - float(self.sensor.trust_car)) * float(object_probability_car)) + (
                        (1 - float(self.sensor.trust_truck)) * float(object_probability_truck)) + (
                                (1 - float(self.sensor.trust_motorcycle)) * float(object_probability_motorcycle)))) + (
                                        (float(object_moved)) * (float(object_probability_vehicle)) * (
                                            ((1 - float(self.sensor.trust_bicycle)) * float(object_probability_bicycle)) + (
                                                (1 - float(self.sensor.trust_pedestrian)) * float(object_probability_pedestrian))))

        self.mass_vehicle_stationary = (1 - float(object_moved)) * (float(object_probability_vehicle)) * (
                    ((1 - float(self.sensor.trust_car)) * float(object_probability_car)) + (
                        (1 - float(self.sensor.trust_truck)) * float(object_probability_truck)) + (
                                (1 - float(self.sensor.trust_motorcycle)) * float(object_probability_motorcycle)))
        self.mass_vru_stationary = (1 - float(object_moved)) * (float(object_probability_vru)) * (
                    ((1 - float(self.sensor.trust_bicycle)) * float(object_probability_bicycle)) + (
                        (1 - float(self.sensor.trust_pedestrian)) * float(object_probability_pedestrian)))

        self.mass_ignorance = (1 - (
                    self.mass_car + self.mass_truck + self.mass_motorcycle + self.mass_bicycle + self.mass_pedestrian + self.mass_stationary + self.mass_vehicle + self.mass_vru + self.mass_traffic + self.mass_vehicle_stationary + self.mass_vru_stationary))

        self.list_classification_mass_factor = [self.mass_car, self.mass_truck, self.mass_motorcycle, self.mass_bicycle,
                                                self.mass_pedestrian, self.mass_stationary, self.mass_vehicle,
                                                self.mass_vru, self.mass_traffic, self.mass_vehicle_stationary,
                                                self.mass_vru_stationary, self.mass_ignorance]
        self.list_classification_mass_factor = [1 if i > 1 else i for i in self.list_classification_mass_factor]

        #print("mass vehicle",self.mass_vehicle)
        #print("mass vru",self.mass_vru)
        #print("mass traffic",self.mass_traffic)
        #print("mass veh station",self.mass_vehicle_stationary)
        #print("mass vru station",self.mass_vru_stationary)
        #print("mass  ignorance",self.mass_ignorance)

