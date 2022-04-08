import numpy as np
from scipy.spatial import distance as di
from scipy.stats import chi2
def statistical_distance(sensor_association_state, global_association_state, sensor_covariance, global_covariance):
    """
    Fusion to calculate the statistical distance between the sensor object and global objects NOT based on feature points .
    calculated based on Central coordinates
    (FOR TESTING ONLY)

    """
    C = np.array([[1, 0], [0, 1]])
    c_m = np.array([[0.5, 0], [0, 0.5]])
    innov_cov = (C.dot(sensor_covariance)).dot(C.transpose()) + c_m


    distance = di.mahalanobis(sensor_association_state, global_association_state, innov_cov)
    threshold = chi2.ppf(0.95, len(sensor_association_state))
    
    return distance,threshold

