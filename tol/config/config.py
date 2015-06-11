class Config(object):
    """
    Configuration options
    """

    def __init__(self, **kwargs):
        """
        :param kwargs:
        :return:
        """
        self.update_rate = kwargs.get('update_rate', 5)
        self.visualize_sensors = kwargs.get('visualize_sensors', False)

        # Original parameter values from RobogenCollision.cpp
        self.surface_friction1 = 1.0
        self.surface_friction2 = 1.0
        self.surface_slip1 = 0.01
        self.surface_slip2 = 0.01
        self.surface_soft_cfm = 0.00001
        self.surface_soft_erp = 0.2
