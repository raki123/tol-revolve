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
