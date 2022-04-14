import object_fusion_msgs.msg

class Dimension(object):
    """Dimension is a struct to hold Dimension Data."""
    def __init__(self,length=0,width=0,length_variance=0,width_variance=0):
        super(Dimension, self).__init__()
        self.length = length
        self.width = width
        self.length_variance = length_variance
        self.width_variance = width_variance

    def to_ros_msg(self):
        return object_fusion_msgs.msg.Dimension(length          = self.length,
                                                width           = self.width,
                                                length_variance = self.length_variance,
                                                width_variance  = self.width_variance)

    def from_ros_message(self, ros_msg_data):  
        self.length          = ros_msg_data.length
        self.width           = ros_msg_data.width
        self.length_variance = ros_msg_data.length_variance
        self.width_variance  = ros_msg_data.width_variance
        return self

        pass