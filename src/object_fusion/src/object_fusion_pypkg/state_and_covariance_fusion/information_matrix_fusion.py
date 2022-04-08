import rospy
import numpy as np
from scipy.stats import chi2
from scipy.linalg import sqrtm
from scipy.spatial import distance as di
from object_fusion_msgs.msg import Geometric

def information_matrix_fusion(glob_pred_obj,prev_obj_aligned,predict_obj,sensor_id,sensortype):
    """
    Function to perfrom the Information matrix fusion.

    """
    global_state_matrix = np.array([[float(glob_pred_obj.geometric.x)] , [float(glob_pred_obj.geometric.vx)],
                                    [float(glob_pred_obj.geometric.ax)], [float(glob_pred_obj.geometric.y)],
                                    [float(glob_pred_obj.geometric.vy)], [float(glob_pred_obj.geometric.ay)]])
    _id = np.zeros((6, 6))
    np.fill_diagonal(_id, 1)
    global_cvarience_matrix = np.reshape(glob_pred_obj.covariance, (6, 6)) + _id
    id2 = np.zeros((6,6))

    sensor_state_matrix = np.array([[float(predict_obj.geometric.x)], [float(predict_obj.geometric.vx)],
                                         [float(predict_obj.geometric.ax)],
                                         [float(predict_obj.geometric.y)], [float(predict_obj.geometric.vy)],
                                         [float(predict_obj.geometric.ay)]])
    sensor_covarience_matrix = np.reshape(predict_obj.covariance, (6, 6))#+id2

    previous_sensor_state_matrix = np.array(
                                        [[float(prev_obj_aligned.geometric.x)], [float(prev_obj_aligned.geometric.vx)],
                                         [float(prev_obj_aligned.geometric.ax)], [float(prev_obj_aligned.geometric.y)],
                                         [float(prev_obj_aligned.geometric.vy)], [float(prev_obj_aligned.geometric.ay)]])
    previous_sensor_covarience_matrix = np.reshape(prev_obj_aligned.covariance, (6, 6))

    try:
        glob_pred_obj_inv = np.linalg.inv(global_cvarience_matrix)
    except:
        glob_pred_obj_inv = np.linalg.pinv(global_cvarience_matrix)

    try:
        sensor_covarience_matrix_inv = np.linalg.inv(sensor_covarience_matrix)
    except:
        sensor_covarience_matrix_inv = np.linalg.pinv(sensor_covarience_matrix)

    try:
        previous_sensor_covarience_matrix_inv = np.linalg.inv(previous_sensor_covarience_matrix)
    except:
        previous_sensor_covarience_matrix_inv = np.linalg.pinv(previous_sensor_covarience_matrix)

    inverse_fused_covarience_matrix = (glob_pred_obj_inv) + ((sensor_covarience_matrix_inv) - previous_sensor_covarience_matrix_inv)
    fused_covarience_matrix = np.linalg.pinv(inverse_fused_covarience_matrix)

    fused_state_matrix = fused_covarience_matrix.dot(
        (glob_pred_obj_inv.dot(global_state_matrix)) + (sensor_covarience_matrix_inv.dot(sensor_state_matrix))
        - previous_sensor_covarience_matrix_inv.dot(previous_sensor_state_matrix))

    return [Geometric(x  = float(fused_state_matrix[0]),
                      vx = float(fused_state_matrix[1]),
                      ax = float(fused_state_matrix[2]),
                      y  = float(fused_state_matrix[3]),
                      vy = float(fused_state_matrix[4]),
                      ay = float(fused_state_matrix[5])), fused_covarience_matrix]