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
from object_fusion_msgs.msg import Classification

class ClassificationFusion:
    """
    Class to perform classification fusion between the associated objects.

    Class Attributes:
    mass_factor_combination

    Class Methods:
    __init__
    classification_fusion
    """

    mass_factor_combination = [
        ['car', 'null', 'null', 'null', 'null', 'null', 'car', 'null', 'car', 'car', 'null', 'car'],
        ['null', 'truck', 'null', 'null', 'null', 'null', 'truck', 'null', 'truck', 'truck', 'null', 'truck'],
        ['null', 'null', 'moto', 'null', 'null', 'null', 'moto', 'null', 'moto', 'moto', 'null', 'moto'],
        ['null', 'null', 'null', 'bicycle', 'null', 'null', 'null', 'bicycle', 'bicycle', 'null', 'bicycle', 'bicycle'],
        ['null', 'null', 'null', 'null', 'ped', 'null', 'null', 'ped', 'ped', 'null', 'ped', 'ped'],
        ['null', 'null', 'null', 'null', 'null', 'stat', 'null', 'null', 'null', 'stat', 'stat', 'stat'],
        ['car', 'truck', 'moto', 'null', 'null', 'null', 'veh', 'null', 'veh', 'veh', 'null', 'veh'],
        ['null', 'null', 'null', 'bicycle', 'ped', 'null', 'null', 'vru', 'vru', 'null', 'vru', 'vru'],
        ['car', 'truck', 'moto', 'bicycle', 'ped', 'null', 'veh', 'vru', 'traffic', 'veh', 'vru', 'traffic'],
        ['car', 'truck', 'moto', 'null', 'null', 'stat', 'veh', 'null', 'veh', 'vehstat', 'stat', 'vehstat'],
        ['null', 'null', 'null', 'bicycle', 'ped', 'stat', 'null', 'vru', 'vru', 'stat', 'vrustat', 'vrustat'],
        ['car', 'truck', 'moto', 'bicycle', 'ped', 'stat', 'veh', 'vru', 'traffic', 'vehstat', 'vrustat', 'all']]

    def __init__(self, sensor_object, global_object):
        """
        Class object constructor

        :param sensor_object: Sensor objects involved in the classification fusion
        :param global_object: Global object involved in the classification fusion.
        """
        self.sensor_object = sensor_object
        self.global_object = global_object

        self.fused_mass_factor_car = None
        self.fused_mass_factor_truck = None
        self.fused_mass_factor_motorcycle = None
        self.fused_mass_factor_bicycle = None
        self.fused_mass_factor_pedestrian = None
        self.fused_mass_factor_stationary = None
        self.fused_mass_factor_vehicle = None
        self.fused_mass_factor_vru = None
        self. fused_mass_factor_traffic = None
        self. fused_mass_factor_statvehicle = None
        self.fused_mass_factor_statvru = None
        self. fused_mass_factor_ignorance = None

        self.fused_probability_car = None
        self.fused_probability_truck = None
        self.fused_probability_motorcycle = None
        self.fused_probability_bicycle = None
        self.fused_probability_pedestrian = None
        self.fused_probability_stationary = None
        self.fused_probability_other = None



    def fuse(self):
        """
        Method to perfrom the classification fusion.
        Updates the clssification fusion results in the class object.

        """
        sensor_classification_mass_factors = (self.sensor_object.list_classification_mass_factor)
        global_classification_mass_factors = (self.global_object.list_classification_mass_factor)

        sum_intersection_car = 0
        sum_intersection_truck = 0
        sum_intersection_motorcycle = 0
        sum_intersection_bicycle = 0
        sum_intersection_pedestrian = 0
        sum_intersection_stationary = 0
        sum_intersection_vehicle = 0
        sum_intersection_vru = 0
        sum_intersection_traffic = 0
        sum_intersection_statvehicle = 0
        sum_intersection_statvru = 0
        sum_intersection_null = 0
        sum_intersection_all = 0
        for row in range(len(sensor_classification_mass_factors)):
            for column in range(len(global_classification_mass_factors)):
                if ClassificationFusion.mass_factor_combination[row][column] == 'car':
                    sum_intersection_car = sum_intersection_car + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'truck':
                    sum_intersection_truck = sum_intersection_truck + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'moto':
                    sum_intersection_motorcycle = sum_intersection_motorcycle + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'bicycle':
                    sum_intersection_bicycle = sum_intersection_bicycle + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'ped':
                    sum_intersection_pedestrian = sum_intersection_pedestrian + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'stat':
                    sum_intersection_stationary = sum_intersection_stationary + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'veh':
                    sum_intersection_vehicle = sum_intersection_vehicle + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'vru':
                    sum_intersection_vru = sum_intersection_vru + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'traffic':
                    sum_intersection_traffic = sum_intersection_traffic + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'vehstat':
                    sum_intersection_statvehicle = sum_intersection_statvehicle + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'vrustat':
                    sum_intersection_statvru = sum_intersection_statvru + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'all':
                    sum_intersection_all = sum_intersection_all + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

                elif ClassificationFusion.mass_factor_combination[row][column] == 'null':
                    sum_intersection_null = sum_intersection_null + (
                                sensor_classification_mass_factors[row] * global_classification_mass_factors[column])

        #if sum_intersection_null ==0:
        #    sum_intersection_null=0.1
        self.fused_mass_factor_car = (sum_intersection_car) / (1 - sum_intersection_null)
        self.fused_mass_factor_truck = (sum_intersection_truck) / (1 - sum_intersection_null)
        self.fused_mass_factor_motorcycle = (sum_intersection_motorcycle) / (1 - sum_intersection_null)
        self.fused_mass_factor_bicycle = (sum_intersection_bicycle) / (1 - sum_intersection_null)
        self.fused_mass_factor_pedestrian = (sum_intersection_pedestrian) / (1 - sum_intersection_null)
        self.fused_mass_factor_stationary = (sum_intersection_stationary) / (1 - sum_intersection_null)
        self.fused_mass_factor_vehicle = (sum_intersection_vehicle) / (1 - sum_intersection_null)
        self.fused_mass_factor_vru = (sum_intersection_vru) / (1 - sum_intersection_null)
        self.fused_mass_factor_traffic = (sum_intersection_traffic) / (1 - sum_intersection_null)
        self.fused_mass_factor_statvehicle = (sum_intersection_statvehicle) / (1 - sum_intersection_null)
        self.fused_mass_factor_statvru = (sum_intersection_statvru) / (1 - sum_intersection_null)
        self.fused_mass_factor_ignorance = (sum_intersection_all) / (1 - sum_intersection_null)

        self.fused_probability_car = (self.fused_mass_factor_car) + ((1 / 3) * self.fused_mass_factor_vehicle) + (
                    (1 / 5) * self.fused_mass_factor_traffic) + ((1 / 4) * self.fused_mass_factor_statvehicle)
        self.fused_probability_truck = (self.fused_mass_factor_truck) + ((1 / 3) * self.fused_mass_factor_vehicle) + (
                    (1 / 5) * self.fused_mass_factor_traffic) + ((1 / 4) * self.fused_mass_factor_statvehicle)
        self.fused_probability_motorcycle = (self.fused_mass_factor_motorcycle) + ((1 / 3) * self.fused_mass_factor_vehicle) + (
                    (1 / 5) * self.fused_mass_factor_traffic) + ((1 / 4) * self.fused_mass_factor_statvehicle)
        self.fused_probability_bicycle = (self.fused_mass_factor_bicycle) + ((1 / 2) * self.fused_mass_factor_vru) + (
                    (1 / 5) * self.fused_mass_factor_traffic) + ((1 / 3) * self.fused_mass_factor_statvru)
        self.fused_probability_pedestrian = (self.fused_mass_factor_pedestrian) + ((1 / 2) * self.fused_mass_factor_vru) + (
                    (1 / 5) * self.fused_mass_factor_traffic) + ((1 / 3) * self.fused_mass_factor_statvru)
        self.fused_probability_stationary = (self.fused_mass_factor_stationary) + (
                    (1 / 3) * self.fused_mass_factor_statvehicle) + ((1 / 3) * self.fused_mass_factor_statvru)
        self.fused_probability_other = 1 - (self.fused_probability_car + self.fused_probability_truck + self.fused_probability_motorcycle + self.fused_probability_bicycle + self.fused_probability_pedestrian + self.fused_probability_stationary)

    def get_fused_classification_massfactors_list(self):
        return [self.fused_mass_factor_car,
                self.fused_mass_factor_truck,
                self.fused_mass_factor_motorcycle,
                self.fused_mass_factor_bicycle,
                self.fused_mass_factor_pedestrian,
                self.fused_mass_factor_stationary,
                self.fused_mass_factor_vehicle,
                self.fused_mass_factor_vru,
                self.fused_mass_factor_traffic,
                self.fused_mass_factor_statvehicle,
                self.fused_mass_factor_statvru,
                self.fused_mass_factor_ignorance]

    def get_classification_probabilities(self):
        return Classification(car  = self.fused_probability_car,
                             truck = self.fused_probability_truck,
                        motorcycle = self.fused_probability_motorcycle,
                           bicycle = self.fused_probability_bicycle,
                        pedestrian = self.fused_probability_pedestrian,
                        stacionary = self.fused_probability_stationary,
                             other = self.fused_probability_other)