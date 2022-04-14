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

from ..ros2python.Objects import Objects
class Tracked_Object(object):
    """docstring for Track"""
    def __init__(self,obj_id,current_object,current_timestamp,parent_sensor):
        super(Tracked_Object, self).__init__()
        self.current_object = current_object
        self.ids = self.obj_id = obj_id
        self.current_timestamp = current_timestamp
        self.parent_sensor = parent_sensor
        self.updated = True

    def add_sensor_objects(self, add_objects: Objects, timestamp: float):        
        self.previous_object = self.current_object
        self.previous_timestamp = self.current_timestamp
        self.current_object = add_objects
        self.current_timestamp = timestamp
        self.updated = True

    def __str__(self):
        return f"Tracked_Object Id: {self.current_object.obj_id}"

    def set_existance_probability_mass_factors(self,sensor_property):
        """
        Calculate the exitence probability mass factors for the sensor object using the existence vector.
        """

        self.current_object.classification_mass.mass_existance = (float(self.current_object.prop_persistance) * float(sensor_property.trust_existance) * float(self.current_object.prop_existence))
        self.current_object.classification_mass.mass_nonexistance = (float(self.current_object.prop_persistance) * float(sensor_property.trust_existance) * float((1 - self.current_object.prop_existence)))
        self.current_object.classification_mass.mass_uncertainity = (1 - (float(self.current_object.classification_mass.mass_existance) + float(self.current_object.classification_mass.mass_nonexistance)))

        self.current_object.classification_mass.list_existance_mass_factor = [self.current_object.classification_mass.mass_existance, self.current_object.classification_mass.mass_nonexistance, self.current_object.classification_mass.mass_uncertainity]

    def set_classification_mass_factors(self, sensor_property):
        """
        Claculate the classification probability mass factors for the sensor object using the classification vector.
        """
        object_moved = 0.5

        self.current_object.classification_mass.mass_car = float(sensor_property.trust_car) * float(self.current_object.classification.car)
        self.current_object.classification_mass.mass_truck = float(sensor_property.trust_truck) * float(self.current_object.classification.truck)
        self.current_object.classification_mass.mass_motorcycle = float(sensor_property.trust_motorcycle) * float(self.current_object.classification.motorcycle)
        self.current_object.classification_mass.mass_bicycle = float(sensor_property.trust_bicycle) * float(self.current_object.classification.bicycle)
        self.current_object.classification_mass.mass_pedestrian = float(sensor_property.trust_pedestrian) * float(self.current_object.classification.pedestrian)
        self.current_object.classification_mass.mass_stationary = float(sensor_property.trust_stationary) * float(self.current_object.classification.stationary)

        mass_sum = float(self.current_object.classification_mass.mass_car) + float(self.current_object.classification_mass.mass_truck) + float(self.current_object.classification_mass.mass_motorcycle) + float(self.current_object.classification_mass.mass_bicycle) + float(self.current_object.classification_mass.mass_pedestrian)
        if mass_sum == 0:
            mass_sum = 0.1

        object_probability_vehicle = (float(self.current_object.classification_mass.mass_car) + float(self.current_object.classification_mass.mass_truck) + float(self.current_object.classification_mass.mass_motorcycle)) / (mass_sum)
        object_probability_vru = (float(self.current_object.classification_mass.mass_bicycle) + float(self.current_object.classification_mass.mass_pedestrian)) / (mass_sum)

        self.current_object.classification_mass.mass_vehicle = (float(object_moved)) * (float(object_probability_vehicle)) * (
                    ((1 - float(sensor_property.trust_car)) * float(self.current_object.classification.car)) + (
                        (1 - float(sensor_property.trust_truck)) * float(self.current_object.classification.truck)) + (
                                (1 - float(sensor_property.trust_motorcycle)) * float(self.current_object.classification.motorcycle)))
        self.current_object.classification_mass.mass_vru = (float(object_moved)) * (float(object_probability_vru)) * (
                    ((1 - float(sensor_property.trust_bicycle)) * float(self.current_object.classification.bicycle)) + (
                        (1 - float(sensor_property.trust_pedestrian)) * float(self.current_object.classification.pedestrian)))
        self.current_object.classification_mass.mass_traffic = ((float(object_moved)) * (float(object_probability_vru)) * (
                    ((1 - float(sensor_property.trust_car)) * float(self.current_object.classification.car)) + (
                        (1 - float(sensor_property.trust_truck)) * float(self.current_object.classification.truck)) + (
                                (1 - float(sensor_property.trust_motorcycle)) * float(self.current_object.classification.motorcycle)))) + (
                                        (float(object_moved)) * (float(object_probability_vehicle)) * (
                                            ((1 - float(sensor_property.trust_bicycle)) * float(self.current_object.classification.bicycle)) + (
                                                (1 - float(sensor_property.trust_pedestrian)) * float(self.current_object.classification.pedestrian))))

        self.current_object.classification_mass.mass_vehicle_stationary = (1 - float(object_moved)) * (float(object_probability_vehicle)) * (
                    ((1 - float(sensor_property.trust_car)) * float(self.current_object.classification.car)) + (
                        (1 - float(sensor_property.trust_truck)) * float(self.current_object.classification.truck)) + (
                                (1 - float(sensor_property.trust_motorcycle)) * float(self.current_object.classification.motorcycle)))
        self.current_object.classification_mass.mass_vru_stationary = (1 - float(object_moved)) * (float(object_probability_vru)) * (
                    ((1 - float(sensor_property.trust_bicycle)) * float(self.current_object.classification.bicycle)) + (
                        (1 - float(sensor_property.trust_pedestrian)) * float(self.current_object.classification.pedestrian)))

        self.current_object.classification_mass.mass_ignorance = (1 - (self.current_object.classification_mass.mass_car + self.current_object.classification_mass.mass_truck + self.current_object.classification_mass.mass_motorcycle + self.current_object.classification_mass.mass_bicycle + self.current_object.classification_mass.mass_pedestrian + self.current_object.classification_mass.mass_stationary + self.current_object.classification_mass.mass_vehicle + self.current_object.classification_mass.mass_vru + self.current_object.classification_mass.mass_traffic + self.current_object.classification_mass.mass_vehicle_stationary + self.current_object.classification_mass.mass_vru_stationary))

        self.current_object.classification_mass.list_classification_mass_factor = [self.current_object.classification_mass.mass_car, self.current_object.classification_mass.mass_truck, self.current_object.classification_mass.mass_motorcycle, self.current_object.classification_mass.mass_bicycle,
                                                self.current_object.classification_mass.mass_pedestrian, self.current_object.classification_mass.mass_stationary, self.current_object.classification_mass.mass_vehicle,
                                                self.current_object.classification_mass.mass_vru, self.current_object.classification_mass.mass_traffic, self.current_object.classification_mass.mass_vehicle_stationary,
                                                self.current_object.classification_mass.mass_vru_stationary, self.current_object.classification_mass.mass_ignorance]
        
        self.current_object.classification_mass.list_classification_mass_factor = [1 if i > 1 else i for i in self.current_object.classification_mass.list_classification_mass_factor]
