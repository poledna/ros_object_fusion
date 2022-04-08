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
import numpy as np
from scipy.stats import chi2
from scipy.linalg import sqrtm
from scipy.spatial import distance as di
from object_fusion_msgs.msg import Geometric

def cross_covariance_recursion_fusion(glob_pred_obj,predict_obj):
    "Function to perform cross covariance recursion fusion on object list "

    global_state_matrix = np.array([[float(glob_pred_obj.geometric.x)], [float(glob_pred_obj.geometric.vx)],
                                   [float(glob_pred_obj.geometric.ax)], [float(glob_pred_obj.geometric.y)],
                                   [float(glob_pred_obj.geometric.vy)], [float(glob_pred_obj.geometric.ay)]])
    global_cvarience_matrix = np.reshape(glob_pred_obj.covariance, (6, 6))

    sensor_state_matrix = np.array([[float(predict_obj.geometric.x)], [float(predict_obj.geometric.vx)],
                                    [float(predict_obj.geometric.ax)],
                                    [float(predict_obj.geometric.y)], [float(predict_obj.geometric.vy)],
                                    [float(predict_obj.geometric.ay)]])
    sensor_covarience_matrix = np.reshape(predict_obj.covariance, (6, 6))

    global_covariance_inv = np.linalg.pinv(global_cvarience_matrix)
    sensor_covariance_inv = np.linalg.pinv(sensor_covarience_matrix)


    inverse_fused_covarience_matrix = global_covariance_inv + sensor_covariance_inv
    fused_covarience_matrix = np.linalg.pinv(inverse_fused_covarience_matrix)

    fused_state_matrix = fused_covarience_matrix.dot(
        (global_covariance_inv.dot(global_state_matrix)) + (sensor_covariance_inv.dot(sensor_state_matrix)))
    #print("global", global_state_matrix, "sensor", sensor_state_matrix, "fused", fused_state_matrix)


    return [Geometric(x  = float(fused_state_matrix[0]),
                      vx = float(fused_state_matrix[1]),
                      ax = float(fused_state_matrix[2]),
                      y  = float(fused_state_matrix[3]),
                      vy = float(fused_state_matrix[4]),
                      ay = float(fused_state_matrix[5])),
            fused_covarience_matrix]
