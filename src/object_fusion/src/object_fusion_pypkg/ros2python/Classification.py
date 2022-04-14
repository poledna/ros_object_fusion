import object_fusion_msgs.msg

class Classification(object):
    """Classification is a struct to hold Classification Data."""
    def __init__(self,car,truck,motorcycle,bicycle,pedestrian,stationary,other):
        super(Classification, self).__init__()
        self.car = car
        self.truck = truck
        self.motorcycle = motorcycle
        self.bicycle = bicycle
        self.pedestrian = pedestrian
        self.stationary = stationary
        self.other = other

    def to_ros_msg(self):
        return object_fusion_msgs.msg.Classification(car = self.car,
                                                     truck = self.truck,
                                                     motorcycle = self.motorcycle,
                                                     bicycle = self.bicycle,
                                                     pedestrian = self.pedestrian,
                                                     stationary = self.stationary,
                                                     other = self.other)
        pass

    def from_ros_message(self, ros_msg_data):
        super(Objects, self).__init__(car        = ros_msg_data.car,
                                      truck      = ros_msg_data.truck,
                                      motorcycle = ros_msg_data.motorcycle,
                                      bicycle    = ros_msg_data.bicycle,
                                      pedestrian = ros_msg_data.pedestrian,
                                      stationary = ros_msg_data.stationary,
                                      other      = ros_msg_data.other)
        return self

        pass