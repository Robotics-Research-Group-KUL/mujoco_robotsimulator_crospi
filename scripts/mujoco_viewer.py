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

Interaction: when the simulator configuration contains an "external_wrench" object, the forces applied with the mouse
are sent to the simulator, which applies them in its physics: double-click a body to select it, then
  Ctrl + right-drag: pull the body (force),   Ctrl + left-drag: twist the body (torque).
The wrench is a spring between the body and the mouse, with the stiffnesses given as arguments, computed on the
displayed state, and published as geometry_msgs/WrenchStamped: header.frame_id = name of the body, wrench expressed in the frame of
the body with reference point at its origin. A zero wrench is published when the mouse is released.

Contact forces: when the simulator configuration contains visualization/contact_topic, the contact forces computed by
the simulator are drawn as red arrows starting at the contact points, with a length proportional to the force (the
mouse force is drawn in cyan). The arrow shows the force acting on the robot (or, for contacts without the robot, on
the moving body, e.g. a free object): it points into that body, so make the moving bodies transparent to see it
(Rendering > Transparent, or key T). Do not use the contact force display of the MuJoCo viewer itself: the viewer only
mirrors the positions, so its own contact forces are not those of the simulation.

All arguments are required:
  --model-topic <topic>        same as visualization/model_topic in the simulator configuration
  --state-topic <topic>        same as visualization/state_topic in the simulator configuration
  --render-rate <Hz>           rate at which the viewer is refreshed (and the mouse wrench is published)
  --wrench-topic <topic|none>  same as external_wrench/topic in the simulator configuration, or none to disable
  --force-stiffness <N/m>      stiffness of the mouse spring for forces (e.g. 200: 10 cm of drag -> 20 N)
  --torque-stiffness <Nm/rad>  stiffness of the mouse spring for torques (e.g. 20: 0.1 rad of twist -> 2 Nm)
  --contact-topic <topic|none> same as visualization/contact_topic in the simulator configuration, or none to disable
  --contact-arrows <mode>      per_contact (one arrow per contact point) or per_body_pair (one arrow per pair of bodies
                               in contact: sum of the forces, at the force-weighted mean contact point)
  --force-arrow-scale <m/N>    length of the force arrows per newton (e.g. 0.025: 10 N -> 25 cm)
  --force-arrow-width <m>      width of the force arrows (e.g. 0.004)
"""

import argparse
import os
import sys
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
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray, UInt8MultiArray


class MujocoStateMirror(Node):

    def __init__(self, model_topic: str, state_topic: str, wrench_topic, contact_topic):
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
        self._contacts = None
        if contact_topic is not None:
            self.create_subscription(Float64MultiArray, contact_topic, self._on_contacts, 1)
            self.get_logger().info(f"Drawing the contact forces published on '{contact_topic}'.")
        self.wrench_pub = None
        if wrench_topic is not None:
            self.wrench_pub = self.create_publisher(WrenchStamped, wrench_topic, 10)
            self.get_logger().info(f"Publishing the mouse forces on '{wrench_topic}'.")

    def publish_wrench(self, body_name: str, force, torque):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = body_name
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (float(v) for v in force)
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = (float(v) for v in torque)
        self.wrench_pub.publish(msg)

    def _on_model(self, msg: UInt8MultiArray):
        with self._lock:
            self._model_bytes = bytes(msg.data)
            self._model_version += 1
            self._state = None
            self._contacts = None
        self.get_logger().info(f"Received a MuJoCo model ({len(msg.data)} bytes).")

    def _on_contacts(self, msg: Float64MultiArray):
        with self._lock:
            self._contacts = np.asarray(msg.data)

    def get_contacts(self):
        with self._lock:
            return self._contacts

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


def perturbation_wrench(m: mujoco.MjModel, d: mujoco.MjData, pert: mujoco.MjvPerturb,
                        force_stiffness: float, torque_stiffness: float):
    """Wrench of the mouse perturbation on the selected body, or None.

    Translation (Ctrl + right-drag): spring between the grabbed point of the body and the point dragged by the mouse,
    force = force_stiffness * (dragged point - grabbed point), applied at the grabbed point.
    Rotation (Ctrl + left-drag): torque = torque_stiffness * (rotation vector from the body orientation to the
    orientation set by the mouse).
    Returns (body id, force, torque), expressed in the frame of the body with reference point at its origin.
    """
    body = pert.select
    translate = pert.active & mujoco.mjtPertBit.mjPERT_TRANSLATE
    rotate = pert.active & mujoco.mjtPertBit.mjPERT_ROTATE
    if body <= 0 or not (translate or rotate):
        return None
    R = d.xmat[body].reshape(3, 3)
    force_w = np.zeros(3)
    torque_w = np.zeros(3)
    if translate:
        grabbed = d.xpos[body] + R @ pert.localpos
        force_w = force_stiffness * (pert.refselpos - grabbed)
        torque_w += np.cross(grabbed - d.xpos[body], force_w)
    if rotate:
        q_err = np.zeros(4)
        q_body_inv = np.zeros(4)
        mujoco.mju_negQuat(q_body_inv, d.xquat[body])
        mujoco.mju_mulQuat(q_err, pert.refquat, q_body_inv)
        rotvec = np.zeros(3)
        mujoco.mju_quat2Vel(rotvec, q_err, 1.0)    # axis * angle, world frame
        torque_w += torque_stiffness * rotvec
    point_w = d.xpos[body] + R @ pert.localpos if translate else d.xpos[body].copy()
    return body, R.T @ force_w, R.T @ torque_w, point_w, force_w


CONTACT_STRIDE = 8   # [body on which the force acts, other body, px, py, pz, fx, fy, fz]
CONTACT_RGBA = np.array([1.0, 0.1, 0.1, 1.0], dtype=np.float32)
MOUSE_RGBA = np.array([0.1, 0.8, 1.0, 1.0], dtype=np.float32)


def contact_arrows(contacts: np.ndarray, mode: str):
    """Arrows (point, force) of the contact message [time, n, entries...], per contact or per pair of bodies."""
    n = int(contacts[1])
    entries = contacts[2:2 + n * CONTACT_STRIDE].reshape(n, CONTACT_STRIDE)
    if mode == "per_contact":
        return [(e[2:5], e[5:8]) for e in entries]
    pairs = {}
    for e in entries:
        pairs.setdefault((int(e[0]), int(e[1])), []).append(e)
    arrows = []
    for group in pairs.values():
        points = np.array([e[2:5] for e in group])
        forces = np.array([e[5:8] for e in group])
        weights = np.linalg.norm(forces, axis=1)
        point = (weights @ points) / weights.sum() if weights.sum() > 0 else points.mean(axis=0)
        arrows.append((point, forces.sum(axis=0)))
    return arrows


def draw_arrows(scene: mujoco.MjvScene, arrows, scale: float, width: float):
    """Replaces the user geometries of the viewer by the given arrows [(point, force, rgba)]."""
    scene.ngeom = 0
    for point, force, rgba in arrows:
        tip = np.asarray(point) + scale * np.asarray(force)
        if scene.ngeom >= scene.maxgeom or np.linalg.norm(tip - point) < 1e-6:
            continue
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).flatten(), rgba)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width, np.asarray(point, dtype=np.float64), tip)
        scene.ngeom += 1


def print_instructions(args, mouse_forces: bool, contact_forces: bool):
    """Prints a highlighted box with the mouse and keyboard instructions of the viewer (only the enabled features)."""
    lines = ["MUJOCO CROSPI VIEWER: HOW TO INTERACT", ""]
    if mouse_forces:
        lines += ["PUSH THE ROBOT (or an object) WITH THE MOUSE:",
                  "  1. Double-click a body to select it",
                  f"  2. Ctrl + right-drag : pull it (force, {args.force_stiffness:g} N per m of drag)",
                  f"     Ctrl + left-drag  : twist it (torque, {args.torque_stiffness:g} Nm per rad)",
                  "  3. Release the mouse to remove the force", ""]
    if contact_forces:
        lines += [f"CONTACT FORCES: red arrows ({args.force_arrow_scale * 100:g} cm per N)"]
        if mouse_forces:
            lines += ["                mouse force: cyan arrow"]
        lines += ["  The arrows point INTO the robot: press  T  to make it transparent and see them",
                  "  (do not use the key F: MuJoCo's own contact forces are not those of the simulation)", ""]
    lines += ["CAMERA:  Left-drag: rotate   Right-drag: move   Scroll: zoom",
              "PANELS:  Tab: hide/show the left panel   Shift+Tab: hide/show the right panel"]
    width = max(len(line) for line in lines) + 4
    bold_yellow, reset = ("\033[1;93m", "\033[0m") if sys.stdout.isatty() else ("", "")
    print("\n" + bold_yellow + "#" * width)
    for line in lines:
        print("# " + line.ljust(width - 4) + " #")
    print("#" * width + reset + "\n", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model-topic", required=True)
    parser.add_argument("--state-topic", required=True)
    parser.add_argument("--render-rate", required=True, type=float)
    parser.add_argument("--wrench-topic", required=True)
    parser.add_argument("--force-stiffness", required=True, type=float)
    parser.add_argument("--torque-stiffness", required=True, type=float)
    parser.add_argument("--contact-topic", required=True)
    parser.add_argument("--contact-arrows", required=True, choices=["per_contact", "per_body_pair"])
    parser.add_argument("--force-arrow-scale", required=True, type=float)
    parser.add_argument("--force-arrow-width", required=True, type=float)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    wrench_topic = None if args.wrench_topic == "none" else args.wrench_topic
    contact_topic = None if args.contact_topic == "none" else args.contact_topic
    node = MujocoStateMirror(args.model_topic, args.state_topic, wrench_topic, contact_topic)
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

            print_instructions(args, node.wrench_pub is not None, contact_topic is not None)
            with mujoco.viewer.launch_passive(m, d) as viewer:
                pushed_body = None   # name of the body to which a mouse wrench is being applied
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
                    arrows = []
                    if node.wrench_pub is not None:
                        with viewer.lock():
                            wrench = perturbation_wrench(m, d, viewer.perturb, args.force_stiffness,
                                                         args.torque_stiffness)
                        if wrench is not None:
                            body, force, torque, point_w, force_w = wrench
                            arrows.append((point_w, force_w, MOUSE_RGBA))
                            name = m.body(body).name
                            if name:
                                node.publish_wrench(name, force, torque)
                                pushed_body = name
                        elif pushed_body is not None:
                            node.publish_wrench(pushed_body, np.zeros(3), np.zeros(3))   # mouse released
                            pushed_body = None
                    contacts = node.get_contacts()
                    if contacts is not None:
                        arrows += [(p, f, CONTACT_RGBA) for p, f in contact_arrows(contacts, args.contact_arrows)]
                    if contact_topic is not None or node.wrench_pub is not None:
                        with viewer.lock():
                            draw_arrows(viewer.user_scn, arrows, args.force_arrow_scale, args.force_arrow_width)
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
