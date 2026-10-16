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
Renders the snapshots of the environments shown in README.md (docs/images/*.png).

The robot (URDF) is attached to each environment (MJCF) as done by the simulator (fixed links kept, mesh paths
resolved, identity base pose) and set at the joint configuration for which the environment was designed.

Usage (from a sourced workspace, needs a display for the OpenGL context, e.g. MUJOCO_GL=glfw):
    python3 docs/generate_snapshots.py
"""

import os
import subprocess

import mujoco
import numpy as np
from PIL import Image

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENV_DIR = os.path.join(PKG, 'config', 'environments')
URDF_DIR = os.path.join(PKG, 'config', 'urdf')
OUT_DIR = os.path.join(PKG, 'docs', 'images')
WIDTH, HEIGHT = 1200, 675

MOVE_HOME = [4.7116, -1.5708, 2.0944, -2.0944, -1.5708, 0.0]
INITIAL = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

# image name: (environment, urdf, joints, [(suffix, lookat, distance, azimuth, elevation), ...])
SNAPSHOTS = {
    'pick_and_place_table_with_box': (
        'pick_and_place/table_with_box.xml', 'ur10e_cylinder_tool.urdf', INITIAL,
        [('', [0.3, -0.5, 0.4], 2.6, 135, -25)]),
    'guarded_motions_table_with_corner': (
        'guarded_motions/table_with_corner.xml', 'ur10e_cylinder_tool.urdf', MOVE_HOME,
        [('', [0.15, -0.55, 0.25], 2.2, 55, -30), ('_closeup', [0.25, -0.56, 0.2], 0.6, 45, -35)]),
    'contour_following_table_with_circular_contour': (
        'contour_following/table_with_circular_contour.xml', 'ur10e_cylinder_tool.urdf', MOVE_HOME,
        [('', [0.15, -0.55, 0.3], 2.2, 140, -28), ('_closeup', [0.174, -0.615, 0.18], 0.8, 110, -45)]),
    'contour_following_table_with_small_sine_contour': (
        'contour_following/table_with_small_sine_contour.xml', 'ur10e_cylinder_tool.urdf', MOVE_HOME,
        [('', [0.15, -0.55, 0.3], 2.2, 140, -28), ('_closeup', [0.174, -0.64, 0.18], 0.8, 110, -45)]),
    'contour_following_table_with_large_sine_contour': (
        'contour_following/table_with_large_sine_contour.xml', 'ur10e_cylinder_tool.urdf', MOVE_HOME,
        [('', [0.15, -0.55, 0.3], 2.2, 140, -28), ('_closeup', [0.174, -0.66, 0.18], 1.1, 110, -45)]),
    'peg_insertion_cylindrical_table_with_hole': (
        'peg_insertion_cylindrical/table_with_hole_1mm_clearance.xml', 'ur10e_cylinder_tool.urdf', MOVE_HOME,
        [('', [0.15, -0.55, 0.3], 2.2, 140, -28), ('_closeup', [0.174, -0.615, 0.2], 0.3, 50, -62)]),
    'peg_insertion_rectangular_table_with_rectangular_hole': (
        'peg_insertion_rectangular/table_with_rectangular_hole_1mm_clearance.xml', 'ur10e_rectangular_tool.urdf', MOVE_HOME,
        [('', [0.15, -0.55, 0.3], 2.2, 140, -28), ('_closeup', [0.174, -0.615, 0.2], 0.3, 50, -62)]),
}

ROBOT_RGBA = [0.75, 0.78, 0.82, 1.0]
TOOL_RGBA = [0.95, 0.55, 0.1, 1.0]


def build(env_file, urdf_file):
    share = subprocess.check_output(['ros2', 'pkg', 'prefix', 'ur_description']).decode().strip()
    urdf = open(os.path.join(URDF_DIR, urdf_file)).read().replace(
        'package://ur_description', share + '/share/ur_description')
    robot = mujoco.MjSpec.from_string(urdf)
    robot.compiler.fusestatic = False
    for g in robot.geoms:
        g.rgba = TOOL_RGBA if g.parent.name in ('cylinder_tool', 'rectangular_tool') else ROBOT_RGBA
    env = mujoco.MjSpec.from_file(os.path.join(ENV_DIR, env_file))
    env.compiler.fusestatic = False
    env.attach(robot, frame=env.worldbody.add_frame(), prefix='')
    env.visual.global_.offwidth = WIDTH
    env.visual.global_.offheight = HEIGHT
    env.visual.quality.shadowsize = 4096
    return env.compile()


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, (env_file, urdf_file, joints, views) in SNAPSHOTS.items():
        m = build(env_file, urdf_file)
        d = mujoco.MjData(m)
        for j, q in zip(['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], joints):
            d.qpos[m.jnt_qposadr[m.joint(j).id]] = q
        mujoco.mj_forward(m, d)
        with mujoco.Renderer(m, HEIGHT, WIDTH) as renderer:
            for suffix, lookat, distance, azimuth, elevation in views:
                cam = mujoco.MjvCamera()
                cam.lookat[:] = lookat
                cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
                renderer.update_scene(d, camera=cam)
                path = os.path.join(OUT_DIR, name + suffix + '.png')
                Image.fromarray(renderer.render()).save(path, optimize=True)
                print('saved', os.path.relpath(path, PKG))


if __name__ == '__main__':
    main()
