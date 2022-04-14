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
    def __init__(self,FL=0,FM=0,FR=0,MR=0,RR=0,RM=0,RL=0,ML=0):
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

    def from_ros_message(self,ros_msg_data):
        self.FL = ros_msg_data.FL
        self.FM = ros_msg_data.FM
        self.FR = ros_msg_data.FR
        self.MR = ros_msg_data.MR
        self.RR = ros_msg_data.RR
        self.RM = ros_msg_data.RM
        self.RL = ros_msg_data.RL
        self.ML = ros_msg_data.ML
        
        return self

        pass