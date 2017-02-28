import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
# tol_path = os.path.abspath(os.path.join(here, '..', '..'))
tol_path = os.path.abspath(os.path.join(here))
# rv_path = os.path.abspath(os.path.join(tol_path, '..', 'revolve'))
rv_path = os.path.abspath(os.path.join(tol_path, '..', 'revolve'))
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from revolve.util import Supervisor
from experiments.online_learn import parser


class OfflineEvolutionSupervisor(Supervisor):
    """
    Supervisor class that adds some output filtering for ODE errors
    """

    def __init__(self, *args, **kwargs):
        super(OfflineEvolutionSupervisor, self).__init__(*args, **kwargs)
        self.ode_errors = 0

    def write_stderr(self, data):
        """
        :param data:
        :return:
        """
        if 'ODE Message 3' in data:
            self.ode_errors += 1
        elif data.strip():
            sys.stderr.write(data)

        if self.ode_errors >= 100:
            self.ode_errors = 0
            sys.stderr.write('ODE Message 3 (100)\n')


args = parser.parse_args()

os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(tol_path, 'build')
os.environ['GAZEBO_MODEL_PATH'] = os.path.join(tol_path, 'res', 'models') + \
                                  ':' + os.path.join(rv_path, 'tools', 'models')

supervisor = OfflineEvolutionSupervisor(
    manager_cmd=[sys.executable, args.manager,
                 "--load-controller", args.load_controller,
                 "--robot-name", args.robot_name,
                 "--experiment-round", args.experiment_round,
                 "--brain-conf-path", args.brain_conf_path],
    analyzer_cmd=os.path.join(rv_path, 'tools', 'analyzer', 'run-analyzer'),
    world_file=os.path.join(here, args.world),
    output_directory=args.output_directory,
    restore_directory=args.restore_directory,
    gazebo_cmd=args.gazebo_cmd,
    manager_args=sys.argv[1:]
)

supervisor.launch()
