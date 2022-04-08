class Sensor_Property(object):
	"""docstring for Sensor_Property"""
	def __init__(self):
		super(Sensor_Property, self).__init__()
		pass

	def from_ros_message(self, ros_msg):

		self.sensor_id = ros_msg.sensor_id
		self.sensortype = ros_msg.sensortype
		self.posx_variance = ros_msg.posx_variance
		self.posy_variance = ros_msg.posy_variance
		self.velx_variance = ros_msg.velx_variance
		self.vely_variance = ros_msg.vely_variance
		self.trust_existance = ros_msg.trust_existance
		self.trust_car = ros_msg.trust_car
		self.trust_truck = ros_msg.trust_truck
		self.trust_motorcycle = ros_msg.trust_motorcycle
		self.trust_bicycle = ros_msg.trust_bicycle
		self.trust_pedestrian = ros_msg.trust_pedestrian
		self.trust_stationary = ros_msg.trust_stationary
		self.trust_other = ros_msg.trust_other
		return self.__class__()

	def create(self, sensor_id,sensortype,posx_variance,posy_variance,velx_variance,vely_variance,trust_existance,trust_car,trust_truck,trust_motorcycle,trust_bicycle,trust_pedestrian,trust_stationary,trust_other):
		self.sensor_id = sensor_id
		self.sensortype = sensortype
		self.posx_variance = posx_variance
		self.posy_variance = posy_variance
		self.velx_variance = velx_variance
		self.vely_variance = vely_variance
		self.trust_existance = trust_existance
		self.trust_car = trust_car
		self.trust_truck = trust_truck
		self.trust_motorcycle = trust_motorcycle
		self.trust_bicycle = trust_bicycle
		self.trust_pedestrian = trust_pedestrian
		self.trust_stationary = trust_stationary
		self.trust_other = trust_other
	
	def to_ros_msg(self):
		from object_list.msg import SensorProperty
		return SensorProperty(self.sensor_id,
							  self.sensortype,
							  self.posx_variance,
							  self.posy_variance,
							  self.velx_variance,
							  self.vely_variance,
							  self.trust_existance,
							  self.trust_car,
							  self.trust_truck,
							  self.trust_motorcycle,
							  self.trust_bicycle,
							  self.trust_pedestrian,
							  self.trust_stationary,
							  self.trust_other)
