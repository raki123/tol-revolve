import argparse


class CustomParser(argparse.ArgumentParser):
    """
    Extends argument parser to add some simple file reading / writing
    functionality.
    """
    def convert_arg_line_to_args(self, arg_line):
        """
        Simple arg line converter that returns `--my-argument value` from
        lines like "my_argument=value"
        :param arg_line:
        :return:
        """
        # Empty or comment line
        if not arg_line or arg_line[0] == "#":
            return []

        split = arg_line.find("=")
        if split < 0:
            return [arg_line]

        k, v = "--"+arg_line[:split].replace("_", "-"), arg_line[1+split:]

        # Try to determine if this key is a store constant action, if so
        # return only the key.
        const = False
        for a in self._actions:
            if k in a.option_strings and a.const is not None:
                const = True
                break

        return [k] if const else [k, v]

    def write_to_file(self, args, file):
        """
        Takes the result of `parse_args` and writes it back
        to a file.
        """
        lines = ["%s=%s\n" % (k, args.__dict__[k]) for k in sorted(args.__dict__.keys())]
        with open(file, 'w') as o:
            o.writelines(lines)


def str_to_bool(v):
    """
    :type v: str
    """
    return v.lower() == "true" or v == "1"


def str_to_address(v):
    """
    :type v: str
    """
    if not v:
        return None

    host, port = v.split(":", 1)
    return host, int(port)


parser = CustomParser(fromfile_prefix_chars='@')
parser.add_argument(
    '--sensor-update-rate',
    default=10, type=int,
    help='The rate at which Gazebo sensors are set to update their values.'
)

parser.add_argument(
    '--controller-update-rate',
    default=10, type=int,
    help='The rate at which the `RobotController` is requested to update.'
)

parser.add_argument(
    '--visualize-sensors',
    default=False, type=bool,
    help='Visualize sensors (helpful for debugging purposes)'
)

parser.add_argument(
    '--pose-update-frequency',
    default=50, type=int,
    help="The frequency at which the world is requested to send robot pose"
         " updates (in number of times per *simulation* second)."
)

parser.add_argument(
    '--speed-window',
    default=600, type=int,
    help="Number of position entries considered to calculate robot speed. "
         "The world's pose update rate will determine the number of seconds this entails."
)

parser.add_argument(
    '--min-parts',
    default=3, type=int,
    help="Minimum number of parts in a robot."
)

parser.add_argument(
    '--max-parts',
    default=10, type=int,
    help="Maximum number of parts in a robot."
)

parser.add_argument(
    '--max-inputs',
    default=8, type=int,
    help="Maximum number of inputs (i.e. sensors) in a robot."
)

parser.add_argument(
    '--max-outputs',
    default=8, type=int,
    help="Maximum number of outputs (i.e. sensors) in a robot."
)

parser.add_argument(
    '--body-mutation-epsilon',
    default=0.05, type=float,
    help="Mutation epsilon for robot bodies."
)

parser.add_argument(
    '--brain-mutation-epsilon',
    default=0.05, type=float,
    help="Mutation epsilon for robot neural nets."
)

parser.add_argument(
    '--p-duplicate-subtree',
    default=0.05, type=float,
    help="Probability of duplicating a subtree."
)

parser.add_argument(
    '--p-swap-subtree',
    default=0.05, type=float,
    help="Probability of swapping two subtrees."
)

parser.add_argument(
    '--p-delete-subtree',
    default=0.05, type=float,
    help="Probability of deleting a subtree."
)

parser.add_argument(
    '--p-remove-brain-connection',
    default=0.05, type=float,
    help="Probability of removing a neural network connection."
)

parser.add_argument(
    '--p-delete-hidden-neuron',
    default=0.05, type=float,
    help="Probability of deleting a random hidden neuron."
)

parser.add_argument(
    '--world-address',
    default="127.0.0.1:11345", type=str,
    help="Host:port of the simulator."
)

parser.add_argument(
    '--analyzer-address',
    default="127.0.0.1:11346", type=str,
    help="Host:port of the body analyzer (set to empty string to ignore)."
)

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
parser.add_argument(
    '--output-directory',
    default=None, type=str,
    help="Directory where robot statistics are written."
)

parser.add_argument(
    '--enable-touch-sensor',
    default=True, type=str_to_bool,
    help="Enable / disable the touch sensor in robots."
)

parser.add_argument(
    '--enable-light-sensor',
    default=True, type=str_to_bool,
    help="Enable / disable the light sensor in robots."
)


# class Config(object):
#     """
#     Configuration options
#     """
#
#     def __init__(self, **kwargs):
#         """
#         :param kwargs:
#         :return:
#         """
#         # World options
#         self.update_rate = kwargs.get('update_rate', 10)
#         self.visualize_sensors = kwargs.get('visualize_sensors', False)
#
#         # Arena size, tuple of meters, meters
#         self.arena_size = kwargs.get('arena_size', (30, 30))
#
#         # Number of entries in the robot's speed window. This does
#         # not translate to a number of seconds *exactly*, but you
#         # can assume roughly one entry per pose update message
#         self.speed_window = kwargs.get('speed_window', 600)
#
#         # The distance two robots need to be from each other in order
#         # to consider mating.
#         self.mating_distance = kwargs.get('mating_distance', 1.0)
#
#         # If other speed / my speed > than this threshold, a mating proposal
#         # is accepted.
#         self.proposal_threshold = kwargs.get('proposal_threshold', 0.5)
#
#         # Seconds
#         self.mating_cooldown = kwargs.get('mating_cooldown', 30)
#
#         # Robot specification
#         self.min_parts = kwargs.get('min_parts', 3)
#         self.max_parts = kwargs.get('max_parts', 10)
#         self.max_inputs = kwargs.get('max_inputs', 8)
#         self.max_outputs = kwargs.get('max_outputs', 12)
#
#         # Mutation parameters
#         self.body_mutation_epsilon = kwargs.get('body_mutation_epsilon', 0.05)
#         self.brain_mutation_epsilon = kwargs.get('brain_mutation_epsilon', 0.05)
#         self.p_duplicate_subtree = kwargs.get('p_duplicate_subtree', 0.05)
#         self.p_swap_subtree = kwargs.get('p_swap_subtree', 0.05)
#         self.p_delete_subtree = kwargs.get('p_delete_subtree', 0.05)
#         self.p_remove_brain_connection = kwargs.get('p_remove_brain_connection', 0.05)
#         self.p_delete_hidden_neuron = kwargs.get('p_delete_hidden_neuron', 0.05)
#
#         # Connection addresses. The analyzer can be set to any falsy value
#         # to not connect to an analyzer.
#         self.world_address = kwargs.get('world_address', ("127.0.0.1", 11345))
#         self.analyzer_address = kwargs.get('analyzer_address', ("127.0.0.1", 11346))
#
#         # Directory where robot information will be written. The system writes
#         # two main CSV files:
#         # - The `robots.csv` file containing all the basic robot information, one line
#         #   per robot, in the format
#         #   id,parent1,parent2
#         # - The `poses.csv` file containing each robot pose through time, in the format
#         #   id,sim_time_sec,sim_time_nsec,x,y,z
#         #
#         # Additionally, the `Robot` protobuf message for each bot is written to
#         # a file called `robot_[ID].pb` when the robot is first registered.
#         #
#         # The files are written to a new YYYYMMDDHHIISS directory within the
#         # specified output directory.
#         self.output_directory = kwargs.get('output_directory', None)
#
#         # Whether or not to enable these sensors
#         self.enable_touch_sensor = kwargs.get('enable_touch_sensor', True)
#         self.enable_light_sensor = kwargs.get('enable_light_sensor', True)

