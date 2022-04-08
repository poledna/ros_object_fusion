from .Tracked_Object import Tracked_Object
from ..ros2python.Objects import Objects
class Sensor_Track():
    """docstring for Sensor_Track"""
    def __init__(self,sensor_property):
        super(Sensor_Track, self).__init__()
        self.tracked_objects = {}
        self.sensor_property = sensor_property
        self.new_object = False

    def set_timestamp(self,timestamp):
        self.timestamp = timestamp

    def add_object(self, add_object: Objects):
        if self.is_object_tracked(add_object):
            self.tracked_objects[add_object.obj_id].add_sensor_objects(add_object,self.timestamp)
        else:
            self.tracked_objects[add_object.obj_id] = Tracked_Object(add_object.obj_id,add_object,self.timestamp,self.sensor_property.sensor_id)

    def unset_update_status(self):
        for tracked_objects in self.tracked_objects:
            self.tracked_objects[tracked_objects].updated = False

    def delete_not_updated(self):
        not_updated = [tracked_objects for tracked_objects in self.tracked_objects if not self.tracked_objects[tracked_objects].updated]
        for del_items in not_updated:                    
            del self.tracked_objects[del_items]

    def is_on_track(self,obj_id):
        return obj_id in self.tracked_objects.keys()
            
    def is_object_tracked(self,check_object):
        return check_object.obj_id in self.tracked_objects.keys()
    

    def __str__(self):
        return f"Track from sensor: {self.sensor_property.sensor_id}, with {len(self.tracked_objects.keys())} objects: {list(self.tracked_objects.keys())}"

    def __repr__(self):
        return f"Track from sensor: {self.sensor_property.sensor_id}, with {len(self.tracked_objects.keys())} objects: {list(self.tracked_objects.keys())}"




