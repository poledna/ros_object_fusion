from . import Pos,Vel,Acc
class Ego:
    def __init__(self):
        self.pos = Pos()
        self.pos.x = 0
        self.pos.y = 0
        self.vel = Vel()
        self.vel.x = 0
        self.vel.y = 0
        self.acc = Acc()
        self.acc.x = 0
        self.acc.y = 0
        self.neworientation = 0
        self.oldorientation = 0
        self.oldyaw = 0
        self.newyaw = 0
        self.yawrate = 0
        self.testyaw = 0
        self.testyawrate = 0
        self.t = 0