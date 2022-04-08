from ..ros2python.Objects import Objects
class Tracked_Object(object):
    """docstring for Track"""
    def __init__(self,obj_id,current_object,current_timestamp,parent_sensor):
        super(Tracked_Object, self).__init__()
        self.current_object = current_object
        self.obj_id = obj_id
        self.current_timestamp = current_timestamp
        self.parent_sensor = parent_sensor
        self.updated = True

    def add_sensor_objects(self, add_objects: Objects, timestamp: float):        
        self.previous_object = self.current_object
        self.previous_timestamp = self.current_timestamp
        self.current_object = add_objects
        self.current_timestamp = timestamp
        self.updated = True

    def __str__(self):
        return f"Tracked_Object Id: {self.current_object.obj_id}"