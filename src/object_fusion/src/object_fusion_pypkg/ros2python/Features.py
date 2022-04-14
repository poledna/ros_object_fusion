import object_fusion_msgs.msg

class Features(object):
    """Features is a struct to hold Features Data.
    uint8 FL
uint8 FM
uint8 FR
uint8 MR
uint8 RR
uint8 RM
uint8 RL
uint8 ML"""
    def __init__(self,FL,FM,FR,MR,RR,RM,RL,ML):
        super(Features, self).__init__()
        self.FL = FL
        self.FM = FM
        self.FR = FR
        self.MR = MR
        self.RR = RR
        self.RM = RM
        self.RL = RL
        self.ML = ML

    def to_ros_msg(self):
        return object_fusion_msgs.msg.Features(FL = self.FL,
                                               FM = self.FM,
                                               FR = self.FR,
                                               MR = self.MR,
                                               RR = self.RR,
                                               RM = self.RM,
                                               RL = self.RL,
                                               ML = self.ML)

    def from_ros_message(self, ros_msg_data):
        super(Objects, self).__init__(FL = ros_msg_data.FL,
                                      FM = ros_msg_data.FM,
                                      FR = ros_msg_data.FR,
                                      MR = ros_msg_data.MR,
                                      RR = ros_msg_data.RR,
                                      RM = ros_msg_data.RM,
                                      RL = ros_msg_data.RL,
                                      ML = ros_msg_data.ML)
        return self

        pass