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
    
