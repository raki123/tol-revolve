from revolve.build.util import in_mm


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

        # Arena size, tuple of meters, meters
        self.arena_size = kwargs.get('arena_size', (10, 10))

        # Number of entries in the robot's speed window. This does
        # not translate to a number of seconds *exactly*, but you
        # can assume roughly one entry per pose update message
        self.speed_window = kwargs.get('speed_window', 600)

        # The distance two robots need to be from each other in order
        # to consider mating.
        self.mating_distance = kwargs.get('mating_distance', in_mm(400))

        # If other speed / my speed > than this threshold, a mating proposal
        # is accepted.
        self.proposal_threshold = kwargs.get('proposal_threshold', 0.5)

        # Seconds
        self.mating_cooldown = kwargs.get('mating_cooldown', 30)

        # Robot specification
        self.min_parts = kwargs.get('min_parts', 3)
        self.max_parts = kwargs.get('max_parts', 15)
        self.max_inputs = kwargs.get('max_inputs', 8)
        self.max_outputs = kwargs.get('max_outputs', 12)

        self.world_address = kwargs.get('world_address', ("127.0.0.1", 11345))
        self.analyzer_address = kwargs.get('analyzer_address', ("127.0.0.1", 11346))


