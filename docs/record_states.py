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
Records a running simulation to a .npz file, to render it afterwards with render_gif.py.

Subscribes to the visualization topics of the simulator (as mujoco_viewer.py does) and stores the compiled model
(MJB, key 'mjb') and all the received states [simulation time, qpos...] (key 'states') until SIGINT or SIGTERM.
If the simulator restarts (new model), the states recorded so far are discarded.

All arguments are required:
  --model-topic <topic>   same as visualization/model_topic in the simulator configuration
  --state-topic <topic>   same as visualization/state_topic in the simulator configuration
  --output <file.npz>     recording
"""

import argparse
import signal

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, UInt8MultiArray


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model-topic", required=True)
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--output", required=True)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("mujoco_state_recorder")
    model = {}
    states = []

    def on_model(msg):
        model["mjb"] = bytes(msg.data)
        states.clear()
        node.get_logger().info(f"Received a MuJoCo model ({len(msg.data)} bytes).")

    model_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(UInt8MultiArray, args.model_topic, on_model, model_qos)
    node.create_subscription(Float64MultiArray, args.state_topic, lambda msg: states.append(np.array(msg.data)), 100)

    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(True))
    signal.signal(signal.SIGINT, lambda *_: stop.append(True))
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.1)

    if "mjb" not in model:
        node.get_logger().error("No model received: nothing saved.")
    else:
        np.savez(args.output, mjb=np.frombuffer(model["mjb"], dtype=np.uint8), states=np.array(states))
        print(f"Saved {len(states)} states to {args.output}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
