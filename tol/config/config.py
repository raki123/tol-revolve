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

        self.min_parts = kwargs.get('min_parts', 3)
        self.max_parts = kwargs.get('max_parts', 15)
        self.max_inputs = kwargs.get('max_inputs', 8)
        self.max_outputs = kwargs.get('max_outputs', 12)

