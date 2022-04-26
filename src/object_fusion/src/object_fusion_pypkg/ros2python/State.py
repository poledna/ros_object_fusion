import object_fusion_msgs.msg

class State(object):
    """State is a struct to hold State Data."""
    def __init__(self,x=0,y=0,vx=0,vy=0,ax=0,ay=0,yaw=0):
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ax = ax
        self.ay = ay
        self.yaw = yaw

    def to_ros_msg(self):
        return object_fusion_msgs.msg.State(x = self.x,
                                                     y = self.y,
                                                     vx = self.vx,
                                                     vy = self.vy,
                                                     ax = self.ax,
                                                     ay = self.ay,
                                                     yaw = self.yaw)
        pass

    def from_ros_message(self, ros_msg_data):
        self.x = ros_msg_data.x
        self.y = ros_msg_data.y
        self.vx = ros_msg_data.vx
        self.vy = ros_msg_data.vy
        self.ax = ros_msg_data.ax
        self.ay = ros_msg_data.ay
        self.yaw = ros_msg_data.yaw
        return self