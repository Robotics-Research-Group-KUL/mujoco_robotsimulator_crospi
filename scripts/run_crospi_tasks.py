#!/usr/bin/env python3
#  Copyright (c) 2026 KU Leuven, Belgium
#
#  Author: Santiago Iregui
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
"""
End-to-end test of a crospi simulator: executes a sequence of crospi tasks (with BeTFSM) on a running crospi_node.

Start the crospi_node first with a setup file that uses the MuJoCo simulator, e.g.:
  ros2 run crospi_core crospi_node --ros-args \\
      -p config_file:="\\$[mujoco_robotsimulator_crospi]/config/ur10e_position_servo.setup.json" -p simulation:=true

All arguments are required:
  --task-list <file>            BeTFSM/crospi task list ($[package] paths are supported)
  --tasks <name> [<name> ...]   names of the tasks of the task list, executed in this order
  --repetitions <int>           number of times the sequence of tasks is executed
  --tick-frequency <Hz>         frequency at which the BeTFSM state machine is ticked
  --publish-frequency <Hz>      frequency at which BeTFSM publishes the status of the state machine

The web interface of BeTFSM (ROSRunner serve option) is disabled.
"""

import argparse
import sys

import rclpy
from betfsm import SUCCEED, Repeat, Sequence, get_logger, set_logger
from betfsm_crospi import CrospiTask, load_task_list
from betfsm_ros import BeTFSMNode, ROSRunner


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task-list", required=True)
    parser.add_argument("--tasks", required=True, nargs="+")
    parser.add_argument("--repetitions", required=True, type=int)
    parser.add_argument("--tick-frequency", required=True, type=float)
    parser.add_argument("--publish-frequency", required=True, type=float)
    args, ros_args = parser.parse_known_args()
    # ROSRunner.run() parses sys.argv again: only the remaining (ROS) arguments are passed on.
    sys.argv = [sys.argv[0]] + ros_args

    rclpy.init(args=ros_args)
    node = BeTFSMNode.get_instance("mujoco_robotsimulator_task_runner")
    set_logger("default", node.get_logger())
    set_logger("crospi", node.get_logger())

    blackboard = {}
    load_task_list(args.task_list, blackboard)

    sequence = Sequence("tasks", children=[CrospiTask(name, name) for name in args.tasks])
    state_machine = Repeat("repeat_tasks", args.repetitions, sequence)

    runner = ROSRunner(node, state_machine, blackboard, frequency=args.tick_frequency,
                       publish_frequency=args.publish_frequency, debug=False, display_active=False, serve=False)
    outcome = None
    try:
        outcome = runner.run()
    except KeyboardInterrupt:
        pass
    get_logger().info(f"Finished with outcome {outcome}")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if outcome == SUCCEED else 1


if __name__ == "__main__":
    sys.exit(main())
