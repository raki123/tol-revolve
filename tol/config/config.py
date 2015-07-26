class Config(object):
    """
    Configuration options
    """

    def __init__(self, **kwargs):
        """
        :param kwargs:
        :return:
        """
        # World options
        self.update_rate = kwargs.get('update_rate', 5)
        self.visualize_sensors = kwargs.get('visualize_sensors', False)

        # Window over which average robot speed is maintained
        self.speed_window = kwargs.get('speed_window', 10)

        # Robot specification
        self.min_parts = kwargs.get('min_parts', 3)
        self.max_parts = kwargs.get('max_parts', 15)
        self.max_inputs = kwargs.get('max_inputs', 8)
        self.max_outputs = kwargs.get('max_outputs', 12)

        self.world_address = kwargs.get('world_address', ("127.0.0.1", 11345))
        self.analyzer_address = kwargs.get('analyzer_address', ("127.0.0.1", 11346))


