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

from object_fusion_pypkg import *

import logging
from threading import Lock
import sys
from time import time

class Fusion:
    def __init__(self,mahalanobis_distance_threshold):
        self.egoveh = ego_state.Ego()  # consists of updated ego parameters

        self.globaltrack = track.Global_Track()

        self.sensors = {}   # list of sensor objectlists
        self.mahalanobis_distance_threshold = mahalanobis_distance_threshold
        self.cost_matrix_A = None
        self.cost_matrix_B = None
        self.cost_matrix = None
        self.association_matrix = None
        self.threshold_matrix = None
        self.assignment_matrix = None
        self.assignment_list = []
        self.tolerance = 1  # tolerance for auction algorithm
        self.breakr = 0  # param to break from auction algorithm if auction algorithm takes too long taking too long
        self.time_elapsed = None

        self.logger = logging.getLogger("object_fusion")

        self.lock = Lock()  # for thread safeness

        self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(log_format.Logging_Format())
        self.logger.addHandler(ch)

    def auction_algorithm(self, cost_matrix, tolerance):
        """
        Method to perform auction algorithm for optimal assignment of sensor level objects and global level objects.
        """

        starttime = rospy.get_rostime().to_sec()

        shape_cost_matrix = cost_matrix.shape
        bid_price = np.zeros((1, shape_cost_matrix[1]), dtype=np.int64)
        assignment_matrix = -9999 * \
            np.ones((1, shape_cost_matrix[0]), dtype=np.int64)

        assignment_complete = False

        while not assignment_complete:

            if -9999 in assignment_matrix:
                not_assigned_object_index = np.where(assignment_matrix == -9999)[1]
                sensor_object_index = not_assigned_object_index[0]

                sensor_object_benifit = cost_matrix[sensor_object_index] - bid_price

                maximum_value_index = np.argmax(sensor_object_benifit[0])
                if maximum_value_index in assignment_matrix:
                    already_assigned_object_index = np.where(
                        assignment_matrix == maximum_value_index)[1][0]
                    assignment_matrix[0][sensor_object_index] = maximum_value_index
                    assignment_matrix[0][already_assigned_object_index] = -9999

                    first_best = np.partition(
                        sensor_object_benifit[0].flatten(), -2)[-1]
                    second_best = np.partition(
                        sensor_object_benifit[0].flatten(), -2)[-2]
                    object_new_bidprice = (
                        float(bid_price[0][sensor_object_index]) + (float(first_best) - float(second_best)) + float((tolerance)))
                    try:
                        bid_price[0][sensor_object_index] = object_new_bidprice
                    except Exception as e:
                        self.logger.critical(e)
                        bid_price[0][sensor_object_index] = sys.maxsize
                else:
                    assignment_matrix[0][sensor_object_index] = maximum_value_index
                endtime = rospy.get_rostime().to_sec()
                time = endtime - starttime
                if time > 0.1 or time < 0:
                    return False
            else:
                assignment_complete = True

        return assignment_matrix[0]

    def create_association_matrix(self, shape_association_matrix):
        # intialize Association matrix (M*N) M - objs in sensor track, N - objs in global track
        self.association_matrix = np.zeros(shape_association_matrix)

    def create_cost_matrix_A(self, shape_cost_matrix):
        # intialize Cost matrixA (M*N) M - objs in sensor track, N - objs in global track
        self.cost_matrix_A = np.zeros(shape_cost_matrix)

    def create_threshold_matrix(self, threshold_matrix_shape):
        # intialize Threshold matrixA (M*N) M - objs in sensor track, N - objs in global track
        self.threshold_matrix = np.zeros(threshold_matrix_shape)

    def create_cost_matrix_B(self, cost_matrix_b_shape, threshold):
        # intialize Cost matrixB (M*M) M - objs in sensor track
        cost_matrix_B = np.identity(cost_matrix_b_shape) * threshold
        self.cost_matrix_B = cost_matrix_B

    def create_cost_matrix(self, sensor_updated):
        for index_global_counter, global_track_index in enumerate(self.globaltrack.tracked_objects):
            global_object = self.globaltrack.tracked_objects[global_track_index].current_fused_object
            for index_sensor_object, sensor_obj_index in enumerate(sensor_updated.tracked_objects):
                sensor_object = sensor_updated.tracked_objects[sensor_obj_index].current_object

                [scenario, globalxf, globalyf, sensorxf, sensoryf,
                    geometric] = feature_select(global_object, sensor_object)

                global_association_state = np.array(
                    [[global_object.geometric.x], [global_object.geometric.y]])
                sensor_association_state = np.array(
                    [[sensor_object.geometric.x], [sensor_object.geometric.y]])

                global_covariance = np.array([[global_object.covariance[0],
                                               global_object.covariance[3]],
                                              [global_object.covariance[18],
                                               global_object.covariance[21]]])
                sensor_covariance = np.array([[sensor_object.covariance[0],
                                               sensor_object.covariance[3]],
                                              [sensor_object.covariance[18],
                                               sensor_object.covariance[21]]])

                maha_distance, threshold = statistical_distance(
                    sensor_association_state, global_association_state, sensor_covariance, global_covariance)

                if maha_distance > threshold:
                    maha_distance = 9999

                self.association_matrix[index_sensor_object,
                                        index_global_counter] = maha_distance
                self.threshold_matrix[index_sensor_object,
                                      index_global_counter] = threshold
                self.cost_matrix_B[index_sensor_object,
                                   index_sensor_object] = threshold

        sensorobjs, globalobjs = np.shape(self.association_matrix)
        for i in range(sensorobjs):
            for j in range(globalobjs):
                if self.association_matrix[i, j] == 9999:
                    self.cost_matrix_A[i, j] = 0
                else:
                    self.cost_matrix_A[i, j] = 2 * self.threshold_matrix[i,j] 
                    self.cost_matrix_A[i, j] -= self.association_matrix[i, j]
        cost_matrix = np.concatenate((self.cost_matrix_A, self.cost_matrix_B), axis=1)       

        return cost_matrix

    def create_assignment_matrix(self, sensor_updated):
        length_sensor_obj_list = len(sensor_updated.tracked_objects.keys())
        length_globaltrack_obj_list = len(
            self.globaltrack.tracked_objects.keys())

        shape_association_matrix = (length_sensor_obj_list, length_globaltrack_obj_list)
        shape_cost_matrix = (length_sensor_obj_list,
                             length_globaltrack_obj_list)
        threshold_matrix_shape = (length_sensor_obj_list, length_globaltrack_obj_list)
        cost_matrix_b_shape = length_sensor_obj_list

        self.create_association_matrix(shape_association_matrix)
        self.create_cost_matrix_A(shape_cost_matrix)
        self.create_threshold_matrix(threshold_matrix_shape)
        self.create_cost_matrix_B(cost_matrix_b_shape, self.mahalanobis_distance_threshold)

        cost_matrix = self.create_cost_matrix(sensor_updated)

        assignment_matrix = self.auction_algorithm(cost_matrix, self.tolerance)

        return assignment_matrix

    def fuse(self, sensor_index_which_needs_fusing):
        self.lock.acquire(blocking=True)
        sensor_updated = self.sensors[sensor_index_which_needs_fusing]

        if len(sensor_updated.tracked_objects) == 0:
            self.logger.warning("NO OBJECTS SEND BY SENSOR")
            self.lock.release()
            return "Sensor did not send Objects"

        assignment_matrix = self.create_assignment_matrix(sensor_updated)
        if not isinstance(assignment_matrix, np.ndarray):
            self.logger.error("BREAKING AUCTION ALGORITHM")
            self.time_penalizer()
            
            self.globaltrack.timestamp = list(sensor_updated.tracked_objects.values())[-1].current_timestamp
            
            self.lock.release()
            return False

        self.logger.debug(f"assignment_matrix{ assignment_matrix} , len global track { len(self.globaltrack.tracked_objects)}, len objlist {len(sensor_updated.tracked_objects.keys())} ")

        sensor_keys = list(sensor_updated.tracked_objects.keys())
        global_keys = list(self.globaltrack.tracked_objects.keys())

        for l, assign in enumerate(assignment_matrix):
            if assign < len(self.globaltrack.tracked_objects):
                sensor_updated.tracked_objects[sensor_keys[l]].fusion_id = self.globaltrack.tracked_objects[global_keys[assign]].id
                sensor_updated.tracked_objects[sensor_keys[l]].new_object = False
            else:
                sensor_updated.tracked_objects[sensor_keys[l]].new_object = True
                self.logger.warning("assignment_matrix outside global size")

        t = sensor_updated.timestamp - self.globaltrack.timestamp
        globaltrack_predicted = temporal_and_spatial_alignment.align_list(
            self.globaltrack.tracked_objects, self.egoveh, t)

        sensor_type = sensor_updated.sensor_property.sensortype

        # list of global object ids
        global_ids = [globaltrack_predicted[index].id for index in globaltrack_predicted.keys()]

        prev_obj_ids = []
        for tracked_object in self.sensors[sensor_index_which_needs_fusing].tracked_objects.values():
            if hasattr(tracked_object, "previous_object"):
                prev_obj_ids.append(tracked_object.previous_object.obj_id)

        for n, sensor_object in enumerate(sensor_updated.tracked_objects.values()):
            if hasattr(sensor_object, "fusion_id"):
                object_fusion_id = sensor_object.fusion_id
            else:
                self.logger.info(f"new_object {sensor_object.obj_id}")
                sensor_object.new_object = True
                object_fusion_id = False

            sensor_trust = sensor_updated.sensor_property.trust_existance
            sensor_property = sensor_updated.sensor_property
            current_object_id = sensor_object.current_object.obj_id

            if object_fusion_id in global_ids and current_object_id in prev_obj_ids:
                self.fusion_if_already_in_global_and_sensor_track(globaltrack_predicted[object_fusion_id],
                                                                  sensor_object,
                                                                  sensor_trust,
                                                                  sensor_property,
                                                                  object_fusion_id)
                self.logger.info(f"SENSOR OBJECT ALREADY EXIST IN GLOBAL TRACK AND PREVIOUS SENSOR TRACK")

            elif object_fusion_id in global_ids and current_object_id not in prev_obj_ids:
                self.fusion_if_already_in_global_but_not_sensor_track(globaltrack_predicted[object_fusion_id],
                                                                      sensor_object,
                                                                      sensor_trust,
                                                                      sensor_property,
                                                                      object_fusion_id)
                self.logger.info(f"1st update from sensor")

            elif sensor_object.new_object:
                self.fusion_new_object(sensor_trust,
                                       sensor_property,
                                       sensor_object,
                                       sensor_updated.timestamp,
                                       sensor_index_which_needs_fusing)
                self.logger.info(f"FUSION IF NEW OBJECT FROM SENSOR (AS PER ASSOCIATION)")

            self.logger.debug(f"object_fusion_id: {object_fusion_id} global_ids {global_ids} object_fusion_id {object_fusion_id}, prev_obj_ids {prev_obj_ids} obj_id {current_object_id}")

        self.time_penalizer()

        print("#"*110)

        self.globaltrack.timestamp = sensor_object.current_timestamp

        self.lock.release()

        return True

    def time_penalizer(self):
        indexes_to_be_popped = []
        for object_index in self.globaltrack.tracked_objects:
            prop_existence = self.globaltrack.tracked_objects[object_index].current_fused_object.prop_existence

            t = self.globaltrack.timestamp - \
                self.globaltrack.tracked_objects[object_index].timestamp
            if t > 0.5:
                self.logger.debug(f"penalizing object index {object_index} prob. exist = {prop_existence}")
                prop_existence -= 0.1
            if prop_existence < 0:
                indexes_to_be_popped.append(object_index)
                self.logger.warning(f"killing {object_index}")
            else:
                self.globaltrack.tracked_objects[object_index].current_fused_object.prop_existence = prop_existence
        for popping_index in indexes_to_be_popped:
            self.globaltrack.tracked_objects.pop(popping_index)

    def fusion_if_already_in_global_and_sensor_track(self, globaltrack_predicted,
                                                     sensor_object,  
                                                     sensor_trust,
                                                     sensor_property,
                                                     object_fusion_id):

        time = sensor_object.current_timestamp - self.globaltrack.timestamp

        prev_obj_aligned = temporal_and_spatial_alignment.align_obj(
            sensor_object.previous_object, self.egoveh, sensor_property, time)

        sensor_object.set_existance_probability_mass_factors(sensor_property)
        sensor_object.set_classification_mass_factors(sensor_property)

        globaltrack_predicted.set_existance_probability_mass_factors(sensor_trust)
        globaltrack_predicted.existance_mass_prediction(0.01)

        fused_classification = classification_fusion.ClassificationFusion.ClassificationFusion(
            sensor_object.current_object, globaltrack_predicted.current_fused_object)
        fused_classification.fuse()

        # FUSION IF NOT 1ST UPDATE OF THIS PARTICULAR SENSOR IN GLOBAL TRACK
        if sensor_property.sensor_id in globaltrack_predicted.fusing_participants:
            [global_state, global_covariance] = state_and_covariance_fusion.information_matrix_fusion(globaltrack_predicted.current_fused_object,
                                                                                                      prev_obj_aligned,
                                                                                                      sensor_object.current_object,
                                                                                                      sensor_property.sensor_id,
                                                                                                      sensor_property.sensortype)
        # FUSION IF  1ST UPDATE OF THIS PARTICULAR SENSOR IN GLOBAL TRACK
        else:
            [global_state, global_covariance] = state_and_covariance_fusion.cross_covariance_recursion_fusion(globaltrack_predicted.current_fused_object,
                                                                                                              sensor_object.current_object)
        globaltrack_predicted.current_fused_object.prop_existence = existence_fusion.fuse(sensor_object.current_object, globaltrack_predicted.current_fused_object)

        globaltrack_predicted.current_fused_object.classification_mass = ClassificationMass(fused_classification.get_fused_classification_massfactors_list())

        globaltrack_predicted.current_fused_object.classification = fused_classification.get_classification_probabilities()


        global_state.yaw = sensor_object.current_object.geometric.yaw


        globaltrack_predicted.current_fused_object.geometric = global_state
        globaltrack_predicted.current_fused_object.covariance = global_covariance.flatten()

        globaltrack_predicted.time = sensor_object.current_timestamp
        globaltrack_predicted.timestamp = sensor_object.current_timestamp

        # Update sensor id in global track (to check in next iteration)
        if sensor_property.sensor_id not in globaltrack_predicted.fusing_participants:
            globaltrack_predicted.fusing_participants.append(sensor_property.sensor_id)

        self.globaltrack.tracked_objects[object_fusion_id] = globaltrack_predicted

    def fusion_if_already_in_global_but_not_sensor_track(self, globaltrack_predicted,
                                                         sensor_object,  # glob_pred_obj
                                                         sensor_trust,
                                                         sensor_property,
                                                         object_fusion_id):

        [global_state, global_covariance] = state_and_covariance_fusion.cross_covariance_recursion_fusion(globaltrack_predicted.current_fused_object,
                                                                                                          sensor_object.current_object)
        sensor_object = track.Tracked_Object(globaltrack_predicted.current_fused_object.obj_id,
                                          globaltrack_predicted.current_fused_object,
                                          globaltrack_predicted.current_fused_object.time,
                                          globaltrack_predicted.current_fused_object.sensors_fused)

        sensor_object.set_existance_probability_mass_factors(sensor_property)
        sensor_object.set_classification_mass_factors(sensor_property)

        globaltrack_predicted.set_existance_probability_mass_factors(sensor_trust)
        globaltrack_predicted.existance_mass_prediction(0.01)

        fused_classification = classification_fusion.ClassificationFusion.ClassificationFusion(
            sensor_object.current_object, globaltrack_predicted.current_fused_object)
        fused_classification.fuse()

        globaltrack_predicted.current_fused_object.prop_existence = existence_fusion.fuse(sensor_object.current_object, globaltrack_predicted.current_fused_object)
        globaltrack_predicted.current_fused_object.prop_persistance = globaltrack_predicted.current_fused_object.prop_persistance

        globaltrack_predicted.current_fused_object.classification_mass = ClassificationMass(fused_classification.get_fused_classification_massfactors_list())

        globaltrack_predicted.current_fused_object.classification = fused_classification.get_classification_probabilities()


        global_state.yaw = sensor_object.current_object.geometric.yaw


        globaltrack_predicted.current_fused_object.geometric = global_state
        globaltrack_predicted.current_fused_object.covariance = global_covariance.flatten()

        globaltrack_predicted.time = sensor_object.current_timestamp
        globaltrack_predicted.timestamp = sensor_object.current_timestamp

        if sensor_property.sensor_id not in globaltrack_predicted.fusing_participants:
            globaltrack_predicted.fusing_participants.append(
                sensor_property.sensor_id)

        self.globaltrack.tracked_objects[object_fusion_id] = globaltrack_predicted

    def fusion_new_object(self, sensor_trust,
                          sensor_property,
                          predict_obj,
                          time,
                          sensor_index_which_needs_fusing):

        predict_obj.set_existance_probability_mass_factors(sensor_property)
        predict_obj.set_classification_mass_factors(sensor_property)
        self.globaltrack.create_new_global_object(predict_obj.current_object,
                                                  [sensor_index_which_needs_fusing],
                                                  time,
                                                  classification_mass=
                                                  predict_obj.current_object.classification_mass.list_classification_mass_factor)
