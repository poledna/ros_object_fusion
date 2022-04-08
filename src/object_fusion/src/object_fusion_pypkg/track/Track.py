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

from .Tracked_Object import Tracked_Object


class Track(object):
    """docstring for Track"""
    def __init__(self,obj_id,current_object):
        super(Track, self).__init__()
        self.current_object = current_object
        self.obj_id = obj_id

    def add_sensor_objects(self, add_objects):        
        self.previous_object = self.current_object
        self.current_object = add_objects        
    
