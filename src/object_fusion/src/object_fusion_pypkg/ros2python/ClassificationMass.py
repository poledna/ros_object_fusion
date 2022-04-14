class ClassificationMass():
    """docstring for ClassificationMass"""
    def __init__(self, classification_mass):
        super(ClassificationMass, self).__init__()
        self.classification_mass = classification_mass
        self.mass_car = classification_mass[0]
        self.mass_truck = classification_mass[1]
        self.mass_motorcycle = classification_mass[2]
        self.mass_bicycle = classification_mass[3]
        self.mass_pedestrian = classification_mass[4]
        self.mass_stationary = classification_mass[5]
        self.mass_vehicle = classification_mass[6]
        self.mass_vru = classification_mass[7]
        self.mass_traffic = classification_mass[8]
        self.mass_vehicle_stationary = classification_mass[9]
        self.mass_vru_stationary = classification_mass[10]
        self.mass_ignorance = classification_mass[11]
        self.list_classification_mass_factor = classification_mass
        self.mass_existance = None
        self.mass_nonexistance = None
        self.mass_uncertainity = None

    def to_ros_msg(self):
        return [self.mass_car,
                self.mass_truck,
                self.mass_motorcycle,
                self.mass_bicycle,
                self.mass_pedestrian,
                self.mass_stationary,
                self.mass_vehicle,
                self.mass_vru,
                self.mass_traffic,
                self.mass_vehicle_stationary,
                self.mass_vru_stationary,
                self.mass_ignorance]

        