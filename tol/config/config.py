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
        self.arena_size = kwargs.get('arena_size', (30, 30))

        # Number of entries in the robot's speed window. This does
        # not translate to a number of seconds *exactly*, but you
        # can assume roughly one entry per pose update message
        self.speed_window = kwargs.get('speed_window', 600)

        # The distance two robots need to be from each other in order
        # to consider mating.
        self.mating_distance = kwargs.get('mating_distance', 1.0)

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

        # Connection addresses. The analyzer can be set to any falsy value
        # to not connect to an analyzer.
        self.world_address = kwargs.get('world_address', ("127.0.0.1", 11345))
        self.analyzer_address = kwargs.get('analyzer_address', ("127.0.0.1", 11346))

        # Directory where robot information will be written. The system writes
        # two main CSV files:
        # - The `robots.csv` file containing all the basic robot information, one line
        #   per robot, in the format
        #   id,parent1,parent2
        # - The `poses.csv` file containing each robot pose through time, in the format
        #   id,sim_time_sec,sim_time_nsec,x,y,z
        #
        # Additionally, the `Robot` protobuf message for each bot is written to
        # a file called `robot_[ID].pb` when the robot is first registered.
        #
        # The files are written to a new YYYYMMDDHHIISS directory within the
        # specified output directory.
        self.output_directory = kwargs.get('output_directory', None)

