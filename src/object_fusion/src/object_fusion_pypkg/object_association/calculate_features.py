import math
from .Features import Features
import numpy as np

def calculate_features(obj):
    x = Features()  # Import a class with all features as float 0.0
    y = Features()
    # Version 1 do not include the verification of the hided features - Just take into account if it is inside the FOV

    # Calculate the features
    features_check = 0

    if obj.dimension.length != 0:
        tg_wl = math.atan(obj.dimension.width / obj.dimension.length)
    else:
        tg_wl = 0


    hip_wl = 0.5 * math.sqrt(obj.dimension.width ** 2 + obj.dimension.length ** 2)
    beta = obj.geometric.yaw - tg_wl
    psi = obj.geometric.yaw + tg_wl

    x.FL = obj.geometric.x + hip_wl * math.cos(psi)
    y.FL = obj.geometric.y + hip_wl * math.sin(psi)

    x.FR = obj.geometric.x + hip_wl * math.cos(beta)
    y.FR = obj.geometric.y + hip_wl * math.sin(beta)

    x.RR = obj.geometric.x - hip_wl * math.cos(-psi)
    y.RR = obj.geometric.y - hip_wl * math.sin(psi)

    x.RL = obj.geometric.x - hip_wl * math.cos(-beta)
    y.RL = obj.geometric.y - hip_wl * math.sin(beta)

    x.FM = (x.FR + x.FL) / 2
    y.FM = (y.FR + y.FL) / 2

    x.ML = (x.RL + x.FL) / 2
    y.ML = (y.RL + y.FL) / 2

    x.MR = (x.RR + x.FR) / 2
    y.MR = (y.RR + y.FR) / 2

    x.RM = (x.RR + x.RL) / 2
    y.RM = (y.RR + y.RL) / 2

    #if more than two features are availabel
    X = np.asarray( [x.FL, x.FM, x.FR, x.MR, x.RR, x.RM, x.RL, x.ML]) # Vector of x position for the Features
    Y = np.asarray( [y.FL, y.FM, y.FR, y.MR, y.RR, y.RM, y.RL, y.ML]) # Vector of y position for the Features


    return x,y

