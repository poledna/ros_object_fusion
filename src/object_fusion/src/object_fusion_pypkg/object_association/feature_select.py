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
from . import calculate_features
import numpy as np
def feature_select(global_obj, sensor_obj):

    glob_feat_x, glob_feat_y = calculate_features.calculate_features(global_obj)
    sens_feat_x, sens_feat_y = calculate_features.calculate_features(sensor_obj)

    #scenario: 1 for common corner,2 for common side feature, 3 for features on common side, features unrelated
    if global_obj.features.FL == 1 and sensor_obj.features.FL == 1:
        scenario = 1
        globalxf = glob_feat_x.FL
        globalyf = glob_feat_y.FL
        sensorxf = sens_feat_x.FL
        sensoryf = sens_feat_y.FL
        geometric = [0,0]
    elif global_obj.features.FR == 1 and sensor_obj.features.FR == 1:
        scenario = 1
        globalxf = glob_feat_x.FR
        globalyf = glob_feat_y.FR
        sensorxf = sens_feat_x.FR
        sensoryf = sens_feat_y.FR
        geometric = [0,0]
    elif global_obj.features.RL == 1 and sensor_obj.features.RL == 1:
        scenario = 1
        globalxf = glob_feat_x.RL
        globalyf = glob_feat_y.RL
        sensorxf = sens_feat_x.RL
        sensoryf = sens_feat_y.RL
        geometric = [0,0]
    elif global_obj.features.RR == 1 and sensor_obj.features.RR == 1:
        scenario = 1
        globalxf = glob_feat_x.RR
        globalyf = glob_feat_y.RR
        sensorxf = sens_feat_x.RR
        sensoryf = sens_feat_y.RR
        geometric = [0,0]
    elif global_obj.features.FM == 1 and sensor_obj.features.FM == 1:
        scenario = 2
        if abs(global_obj.dimension.width - global_obj.dimension.width) <  (np.sqrt(global_obj.dimension.width_variance) + np.sqrt(sensor_obj.dimension.width_variance)):
            globalxf = glob_feat_x.FM
            globalyf = glob_feat_y.FM
            sensorxf = sens_feat_x.FM
            sensoryf = sens_feat_y.FM
            geometric = [0,0]

        else :
            globalxf = glob_feat_x.FM
            globalyf = glob_feat_y.FM
            sensorxf = sens_feat_x.FM
            sensoryf = sens_feat_y.FM
            geometric = [0,1]

    elif global_obj.features.RM == 1 and sensor_obj.features.RM == 1:
        scenario = 2
        if abs(global_obj.dimension.width - sensor_obj.dimension.width) < (
                np.sqrt(global_obj.dimension.width_variance) + np.sqrt(sensor_obj.dimension.width_variance)):
            globalxf = glob_feat_x.RM
            globalyf = glob_feat_y.RM
            sensorxf = sens_feat_x.RM
            sensoryf = sens_feat_y.RM
            geometric = [0, 0]

        else:
            globalxf = glob_feat_x.RM
            globalyf = glob_feat_y.RM
            sensorxf = sens_feat_x.RM
            sensoryf = sens_feat_y.RM
            geometric = [0,1]

    elif global_obj.features.ML == 1 and sensor_obj.features.ML == 1:
        scenario = 2
        if abs(global_obj.dimension.length - sensor_obj.dimension.length) < (
                np.sqrt(global_obj.dimension.length_variance) + np.sqrt(sensor_obj.dimension.length_variance)):
            globalxf = glob_feat_x.ML
            globalyf = glob_feat_y.ML
            sensorxf = sens_feat_x.ML
            sensoryf = sens_feat_y.ML
            geometric = [1,0]

        else:
            globalxf = glob_feat_x.ML
            globalyf = glob_feat_y.ML
            sensorxf = sens_feat_x.ML
            sensoryf = sens_feat_y.ML
            geometric = [1,0]
    elif global_obj.features.MR == 1 and sensor_obj.features.MR == 1:
        scenario = 2
        if abs(global_obj.dimension.length - sensor_obj.dimension.length) < (
                np.sqrt(global_obj.dimension.length_variance) + np.sqrt(sensor_obj.dimension.length_variance)):
            globalxf = glob_feat_x.MR
            globalyf = glob_feat_y.MR
            sensorxf = sens_feat_x.MR
            sensoryf = sens_feat_y.MR
            geometric = [0, 1]

        else:
            globalxf = glob_feat_x.MR
            globalyf = glob_feat_y.MR
            sensorxf = sens_feat_x.MR
            sensoryf = sens_feat_y.MR
            geometric = [0, 1]
    elif global_obj.features.FL == 1 and sensor_obj.features.FR == 1:
        scenario = 3
        globalxf = glob_feat_x.FL
        globalyf = glob_feat_y.FL
        sensorxf = sens_feat_x.FR
        sensoryf = sens_feat_y.FR
        geometric = [0, 1] # delete y elements since only x is common
    elif global_obj.features.FL == 1 and sensor_obj.features.RL == 1:
        scenario = 3
        globalxf = glob_feat_x.FL
        globalyf = glob_feat_y.FL
        sensorxf = sens_feat_x.RL
        sensoryf = sens_feat_y.RL
        geometric = [1, 0] # delete x elements since only y is common

    elif global_obj.features.FR == 1 and sensor_obj.features.FL == 1:
        scenario = 3
        globalxf = glob_feat_x.FR
        globalyf = glob_feat_y.FR
        sensorxf = sens_feat_x.FL
        sensoryf = sens_feat_y.FL
        geometric = [0,1] # delete y elements since only x is common
    elif global_obj.features.FR == 1 and sensor_obj.features.RR == 1:
        scenario = 3
        globalxf = glob_feat_x.FR
        globalyf = glob_feat_y.FR
        sensorxf = sens_feat_x.RR
        sensoryf = sens_feat_y.RR
        geometric = [1, 0] # delete x elements since only y is common
    elif global_obj.features.RR == 1 and sensor_obj.features.FR == 1:
        scenario = 3
        globalxf = glob_feat_x.RR
        globalyf = glob_feat_y.RR
        sensorxf = sens_feat_x.RR
        sensoryf = sens_feat_x.RR
        geometric = [1, 0]  # delete x elements since only y is common
    elif global_obj.features.RR == 1 and sensor_obj.features.RL == 1:
        scenario = 3
        globalxf = glob_feat_x.RR
        globalyf = glob_feat_y.RR
        sensorxf = sens_feat_x.RL
        sensoryf = sens_feat_y.RL
        geometric = [0, 1]  # delete y elements since only x is common
    elif global_obj.features.RL == 1 and sensor_obj.features.RR == 1:
        scenario = 3
        globalxf = glob_feat_x.RL
        globalyf = glob_feat_y.RL
        sensorxf = sens_feat_x.RR
        sensoryf = sens_feat_y.RR
        geometric = [0, 1]  # delete y elements since only x is common
    elif global_obj.features.RL == 1 and sensor_obj.features.FL == 1:
        scenario = 3
        globalxf = glob_feat_x.RL
        globalyf = glob_feat_y.RL
        sensorxf = sens_feat_x.FL
        sensoryf = sens_feat_y.FL
        geometric = [1, 0]  # delete x elements since only y is common
    else:
        scenario = 4
        #return all feature coordinates
        globalxf =  glob_feat_x
        globalyf = glob_feat_y
        sensorxf = sens_feat_x
        sensoryf = sens_feat_y
        geometric = [0, 0]

    return [scenario,globalxf,globalyf,sensorxf,sensoryf,geometric]