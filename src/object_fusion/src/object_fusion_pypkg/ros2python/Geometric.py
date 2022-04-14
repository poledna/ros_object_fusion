import object_fusion_msgs.msg

class Geometric(object):
    """Geometric is a struct to hold Geometric Data."""
    def __init__(self,x,y,vx,vy,ax,ay,yaw):
        super(Geometric, self).__init__()
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ax = ax
        self.ay = ay
        self.yaw = yaw

    def to_ros_msg(self):
        return object_fusion_msgs.msg.Geometric(x = self.x,
                                                     y = self.y,
                                                     vx = self.vx,
                                                     vy = self.vy,
                                                     ax = self.ax,
                                                     ay = self.ay,
                                                     yaw = self.yaw)
        pass

    def from_ros_message(self, ros_msg_data):
        super(Objects, self).__init__(x = ros_msg_data.x,
                                      y = ros_msg_data.y,
                                      vx = ros_msg_data.vx,
                                      vy = ros_msg_data.vy,
                                      ax = ros_msg_data.ax,
                                      ay = ros_msg_data.ay,
                                      yaw = ros_msg_data.yaw)
        return self

        pass