import numpy as np

from scipy.spatial import distance
from scipy.stats import chi2

def get_statistical_distance(scenario, globalxf, globalyf, sensorxf, sensoryf, geometric, sensor_covariance, global_covariance):
    #from scipy.spatial import distance
    """
    Fusion to calculate the statistical distance between the sensor object and global objects based on feature points .

    """

    C = np.array([[1, 0], [0, 1]])
    c_m = np.array([[1, 0], [0, 1]])

    innov_cov = (C.dot(sensor_covariance)).dot(C.transpose()) + c_m

    if scenario == 1:
        global_association_state = np.array([[globalxf],[globalyf]])
        sensor_association_state = np.array([[sensorxf],[sensoryf]])
        distance = distance.mahalanobis(sensor_association_state, global_association_state, innov_cov) #+ 2 * np.log(
            #np.sqrt(np.linalg.det(innov_cov)))
        threshold = chi2.ppf(0.05, len(sensor_association_state))
    elif scenario == 2:
        if geometric[0] == 0 and geometric[1] == 0:
            global_association_state = np.array([[globalxf], [globalyf]])
            sensor_association_state = np.array([[sensorxf],[sensoryf]])
        elif  geometric[0] == 1:
            global_association_state =  float(globalyf)
            sensor_association_state = float(sensoryf)
            sensor_covariance =sensor_covariance[1,1]
            global_covariance = global_covariance[1,1]
            innov_cov = sensor_covariance + 0.1
        elif geometric[1] == 1:
            global_association_state = float(globalxf)
            sensor_association_state = float(sensorxf)
            sensor_covariance = sensor_covariance[0, 0]
            global_covariance = global_covariance[0, 0]
            innov_cov = sensor_covariance + 0.1
        distance = distance.mahalanobis(sensor_association_state, global_association_state, innov_cov)# + 2 * np.log(
            #np.sqrt(np.linalg.det(innov_cov)))
        try:
            threshold = chi2.ppf(0.95, len(sensor_association_state))
        except:
            threshold = chi2.ppf(0.95, 2)
    elif scenario == 3:
        if geometric[0] == 1:
            global_association_state = float(globalyf)
            sensor_association_state = float(sensoryf)
            sensor_covariance = sensor_covariance[1, 1]
            global_covariance = global_covariance[1, 1]
            innov_cov = sensor_covariance + 0.1
        elif geometric[1] == 1:
            global_association_state = float(globalxf)
            sensor_association_state = float(sensorxf)
            sensor_covariance = sensor_covariance[0, 0]
            global_covariance = global_covariance[0, 0]
            innov_cov = sensor_covariance + 0.1
        distance = distance.mahalanobis(sensor_association_state, global_association_state, innov_cov) #+ 2 * np.log(np.sqrt(innov_cov))

        try:
            threshold = chi2.ppf(0.95, len(sensor_association_state))
        except:
            threshold = chi2.ppf(0.95, 1)
    else:
        #globalx = [globalxf.FL, globalxf.FM, globalxf.FR, globalxf.MR, globalxf.RR, globalxf.RM, globalxf.RL,globalxf.ML]  # Vector of x position for the Features / List of objects y(features)
        globalx = [globalxf.FL, globalxf.FM, globalxf.FR, globalxf.MR, globalxf.RR, globalxf.RM, globalxf.RL,globalxf.ML]  # Vector of x position for the Features / List of objects y(features)
        globaly = [globalyf.FL, globalyf.FM, globalyf.FR, globalyf.MR, globalyf.RR, globalyf.RM, globalyf.RL, globalyf.ML]  # Vector of y position for the Features

        sensorx = [sensorxf.FL, sensorxf.FM, sensorxf.FR, sensorxf.MR, sensorxf.RR, sensorxf.RM, sensorxf.RL,
                   sensorxf.ML]  # Vector of x position for the Features / List of objects y(features)
        sensory = [sensoryf.FL, sensoryf.FM, sensoryf.FR, sensoryf.MR, sensoryf.RR, sensoryf.RM, sensoryf.RL,
                   sensoryf.ML]
        distance = float('inf')
        for i in range(len(globalx)):
            global_association_state = np.array([[globalx[i]], [globaly[i]]])
            sensor_association_state = np.array([[sensorx[i]], [sensory[i]]])
            innov_cov = (C.dot(sensor_covariance)).dot(C.transpose()) + c_m

            d = distance.mahalanobis(sensor_association_state, global_association_state, innov_cov) #+ 2 * np.log(
                #np.sqrt(np.linalg.det(innov_cov)))
            if d < distance:
                distance = d
                threshold = chi2.ppf(0.05, len(sensor_association_state))
    return(distance, threshold)
