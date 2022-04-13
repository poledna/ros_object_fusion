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

def align_obj(ob,egoveh,sensor_property,t):
    "function to perform temporal alignment/prediction of single object from objects_list"
    "Returns object with state vector predicted to current time"

    now = rospy.Time.now()
    obj = ob

    id = np.zeros((6, 6))
    np.fill_diagonal(id, 10)

    yaw = egoveh.newyaw

    state = np.array([[float(obj.geometric.x)], [float(obj.geometric.vx)], [float(obj.geometric.ax)], [float(obj.geometric.y)], [float(obj.geometric.vy)],[float(obj.geometric.ay)]])

    a = np.array([[np.cos(yaw), t * np.cos(yaw), t * t * np.cos(yaw) / 2, np.sin(yaw), t * np.sin(yaw),
                   t * t * np.sin(yaw) / 2],
                  [0, np.cos(yaw), t * np.cos(yaw), 0, np.sin(yaw), t * np.sin(yaw)],
                  [0, 0, np.cos(yaw), 0, 0, np.sin(yaw)],
                  [-np.sin(yaw), -t * np.sin(yaw), -t * t * np.sin(yaw) / 2, np.cos(yaw), t * np.cos(yaw),
                   t * t * np.cos(yaw) / 2],
                  [0, -np.sin(yaw), -t * np.sin(yaw), 0, np.cos(yaw), t * np.cos(yaw)],
                  [0, 0, -np.sin(yaw), 0, 0, np.cos(yaw)]])
    u = np.array([[egoveh.vel.x], [egoveh.acc.x], [egoveh.vel.y], [egoveh.acc.y]])
    b = np.array(
        [[-t * np.cos(yaw), -t * t * np.cos(yaw) / 2, -t * np.sin(yaw), -t * t * np.sin(yaw) / 2],
         [0, 0, 0, 0],
         [0, 0, 0, 0],
         [t * np.sin(yaw), t * t * np.sin(yaw), -t * np.cos(yaw), -t * t * np.cos(yaw)],
         [0, 0, 0, 0],
         [0, 0, 0, 0]])
    g = np.array([[t * t * t * np.cos(yaw) / 6, -t * t * t * np.cos(yaw) / 6, t * t * t * np.sin(yaw) / 6,
                   -t * t * t * np.sin(yaw) / 6], [t * t * np.cos(yaw) / 2, 0, t * t * np.sin(yaw) / 2, 0],
                  [t * np.cos(yaw), 0, t * np.sin(yaw), 0],
                  [-t * t * t * np.sin(yaw) / 6, t * t * t * np.sin(yaw) / 6, -t * t * t * np.cos(yaw) / 6,
                   -t * t * t * np.cos(yaw) / 6], [-t * t * np.sin(yaw) / 2, 0, t * t * np.cos(yaw) / 2, 0],
                  [-t * np.sin(yaw), 0, t * np.cos(yaw), 0]])

    c_s =np.array([[0.1, 0, 0, 0],[0, 0, 0, 0],[0, 0, 0.1, 0] ,[0, 0,  0,0]])

    covariance = np.reshape(obj.covariance,(6,6))

    predicted_state = a.dot(state) + b.dot(u)

    predicted_covariance = (a.dot(covariance)).dot(a.transpose()) + id
    obj.covariance = predicted_covariance.flatten()

    return obj