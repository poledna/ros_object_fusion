import object_fusion_msgs.msg

class Classification(object):
    """Classification is a struct to hold Classification Data."""
    def __init__(self,car=0,truck=0,motorcycle=0,bicycle=0,pedestrian=0,stationary=0,other=0):
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
        self.car        = ros_msg_data.car
        self.truck      = ros_msg_data.truck
        self.motorcycle = ros_msg_data.motorcycle
        self.bicycle    = ros_msg_data.bicycle
        self.pedestrian = ros_msg_data.pedestrian
        self.stationary = ros_msg_data.stacionary
        self.other      = ros_msg_data.other
        return self

        pass