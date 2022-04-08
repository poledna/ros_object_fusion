import rospy
import numpy as np
from scipy.stats import chi2
from scipy.linalg import sqrtm
from scipy.spatial import distance as di


def align_list(track,egoveh,t):
    "function to perform temporal alignment/prediction of objects_list"
    "Returns objects list with state vector predicted to current time"
    #global now
    now = rospy.Time.now()
    for i, object_index in enumerate(track):
        obj = track[object_index].current_fused_object
        # print(obj)
        #t = float(now.to_sec()) - float(obj.time)

        yaw = egoveh.yawrate * t


        state = np.array([[float(obj.geometric.x)], [float(obj.geometric.vx)], [float(obj.geometric.ax)], [float(obj.geometric.y)], [float(obj.geometric.vy)],[float(obj.geometric.ay)]])

        a = np.array([[np.cos(yaw), t * np.cos(yaw),t * t * np.cos(yaw) / 2, np.sin(yaw), t * np.sin(yaw),
                       t * t * np.sin(yaw) / 2],
                      [0, np.cos(yaw),  t * np.cos(yaw), 0, np.sin(yaw), t * np.sin(yaw)],
                      [0, 0, np.cos(yaw), 0, 0, np.sin(yaw)],
                      [-np.sin(yaw), -t * np.sin(yaw),-t * t * np.sin(yaw) / 2, np.cos(yaw), t * np.cos(yaw),
                        t * t * np.cos(yaw) / 2],
                      [0, -np.sin(yaw), -t * np.sin(yaw), 0, np.cos(yaw), t * np.cos(yaw)],
                      [0, 0, -np.sin(yaw), 0, 0, np.cos(yaw)]])
        u = np.array([[egoveh.vel.x], [egoveh.acc.x], [egoveh.vel.y], [egoveh.acc.y]])
        b = np.array(
            [[-t * np.cos(yaw),  -t * t * np.cos(yaw) / 2, -t * np.sin(yaw),  -t * t * np.sin(yaw) / 2],
             [0, 0, 0, 0],
             [0, 0, 0, 0],
             [t * np.sin(yaw),  t * t * np.sin(yaw), -t * np.cos(yaw), -t * t * np.cos(yaw)],
             [0, 0, 0, 0],
             [0, 0, 0, 0]])
        g = np.array([[t * t * t * np.cos(yaw) / 6, -t * t * t * np.cos(yaw) / 6, t * t * t * np.sin(yaw) / 6,
                       -t * t * t * np.sin(yaw) / 6], [t * t * np.cos(yaw) / 2, 0, t * t * np.sin(yaw) / 2, 0],
                      [t * np.cos(yaw), 0, t * np.sin(yaw), 0],
                      [-t * t * t * np.sin(yaw) / 6, t * t * t * np.sin(yaw) / 6, -t * t * t * np.cos(yaw) / 6,
                       -t * t * t * np.cos(yaw) / 6], [-t * t * np.sin(yaw) / 2, 0, t * t * np.cos(yaw) / 2, 0],
                      [-t * np.sin(yaw), 0, t * np.cos(yaw), 0]])

        eta_s = np.array([[10],[0],[10] ,[0]])
        id = np.zeros((6, 6))
        np.fill_diagonal(id, 20) # rospy.get_param("Fusion_process_noise")
        covariance = np.reshape(obj.covariance,(6,6))
        predicted_state = a.dot(state) + b.dot(u)
        predicted_covariance = (a.dot(covariance)).dot(a.transpose())+ id #+g.dot(eta_s)#+(g.dot(c_s)).dot(g.transpose())
        #print('COV',obj.obj_id,predicted_covariance,)
        obj.covariance = predicted_covariance.flatten()
        #obj.geometric.x = float(predicted_state[0])
        #obj.geometric.vx = float(predicted_state[1])
        #obj.geometric.ax = float(predicted_state[2])

        #obj.geometric.y = float(predicted_state[3])
        #obj.geometric.vy = float(predicted_state[4])
        #obj.geometric.ay = float(predicted_state[5])


    return track

