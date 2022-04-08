"""
Copyright (C) 2022 Yuri Poledna and Fabio Reway and Redge Castelino and Maikol Drechsler and Shashank Harthi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
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