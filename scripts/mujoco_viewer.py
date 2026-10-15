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
Viewer for mujoco_robotsimulator_crospi.

The simulator (when its configuration contains a "visualization" object) publishes:
  - <model_topic> (std_msgs/UInt8MultiArray, transient local): the compiled MuJoCo model (MJB binary),
  - <state_topic> (std_msgs/Float64MultiArray): [simulation time, qpos...].

This script receives the model, opens the MuJoCo viewer and mirrors the simulated state. The physics are NOT
simulated here: the viewer only displays the state of the simulator running inside crospi (or inside
test_mujoco_robotsimulator). The python package mujoco must have the same version as the MuJoCo library
that the simulator was built with (the MJB format is version specific).

All arguments are required:
  --model-topic <topic>   same as visualization/model_topic in the simulator configuration
  --state-topic <topic>   same as visualization/state_topic in the simulator configuration
  --render-rate <Hz>      rate at which the viewer is refreshed
"""

import argparse
import os
import tempfile
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, UInt8MultiArray


class MujocoStateMirror(Node):

    def __init__(self, model_topic: str, state_topic: str):
        super().__init__("mujoco_robotsimulator_viewer")
        self._lock = threading.Lock()
        self._model_bytes = None
        self._model_version = 0
        self._state = None

        model_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UInt8MultiArray, model_topic, self._on_model, model_qos)
        self.create_subscription(Float64MultiArray, state_topic, self._on_state, 1)
        self.get_logger().info(f"Waiting for the model on '{model_topic}' and the state on '{state_topic}'.")

    def _on_model(self, msg: UInt8MultiArray):
        with self._lock:
            self._model_bytes = bytes(msg.data)
            self._model_version += 1
            self._state = None
        self.get_logger().info(f"Received a MuJoCo model ({len(msg.data)} bytes).")

    def _on_state(self, msg: Float64MultiArray):
        with self._lock:
            self._state = np.asarray(msg.data)

    def get_model(self):
        with self._lock:
            return self._model_version, self._model_bytes

    def pop_state(self):
        with self._lock:
            state, self._state = self._state, None
            return state


def load_model(model_bytes: bytes) -> mujoco.MjModel:
    with tempfile.NamedTemporaryFile(suffix=".mjb", delete=False) as f:
        f.write(model_bytes)
        path = f.name
    try:
        return mujoco.MjModel.from_binary_path(path)
    finally:
        os.remove(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model-topic", required=True)
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--render-rate", required=True, type=float)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MujocoStateMirror(args.model_topic, args.state_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    period = 1.0 / args.render_rate
    loaded_version = 0
    try:
        while rclpy.ok():
            version, model_bytes = node.get_model()
            if model_bytes is None or version == loaded_version:
                time.sleep(0.1)
                continue
            m = load_model(model_bytes)
            d = mujoco.MjData(m)
            loaded_version = version
            node.get_logger().info(f"Loaded the model: nq={m.nq}, nbody={m.nbody}, ngeom={m.ngeom}.")

            with mujoco.viewer.launch_passive(m, d) as viewer:
                while viewer.is_running() and rclpy.ok():
                    if node.get_model()[0] != loaded_version:
                        break  # the simulator was restarted: reload the model
                    state = node.pop_state()
                    if state is not None:
                        if state.size != 1 + m.nq:
                            node.get_logger().error(
                                f"State of size {state.size} does not match the model (expected {1 + m.nq}).")
                        else:
                            with viewer.lock():
                                d.time = state[0]
                                d.qpos[:] = state[1:]
                                mujoco.mj_forward(m, d)
                    viewer.sync()
                    time.sleep(period)
                if not viewer.is_running():
                    break
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
