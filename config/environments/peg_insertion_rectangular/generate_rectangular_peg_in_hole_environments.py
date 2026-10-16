#!/usr/bin/env python3
"""
Generates the rectangular peg-in-hole environments table_with_rectangular_hole_<c>mm_clearance.xml (in the directory
of this script).

The peg is the rectangular tool of urdf/ur10e_rectangular_tool.urdf: cross-section 30 mm (along x of tool0) x 20 mm
(along y of tool0). The clearance is DIAMETRAL in each direction: hole = (30 + c) mm x (20 + c) mm, i.e. c/2 per side.

The hole is placed on the axis of the peg at the MoveHome joints [4.7116, -1.5708, 2.0944, -2.0944, -1.5708, 0] and
aligned with the peg (yaw of tool0 at MoveHome, from the forward kinematics of the UR10e), so that a straight vertical
motion from MoveHome inserts the peg.

Usage: python3 generate_rectangular_peg_in_hole_environments.py <clearance_mm> [<clearance_mm> ...]
       e.g. python3 generate_rectangular_peg_in_hole_environments.py 0.1 0.5 1 2
"""
import math
import os
import sys

PEG_X, PEG_Y = 0.030, 0.020        # cross-section of the peg (along x and y of tool0)
# Axis and yaw of the peg at MoveHome (forward kinematics of urdf/ur10e_*_tool.urdf, identity base pose)
CX, CY = 0.173664, -0.614962
YAW = -0.000789                    # rotation about z of the hole (x of tool0 at MoveHome points along -x of the world,
                                   # rotated by this angle; the hole is symmetric under a rotation of pi)
B = 0.05                           # half side of the square block
DEPTH = 0.05; FLOOR = 0.01         # hole depth, thickness under the hole
ZTOP_TABLE = 0.15

if len(sys.argv) < 2:
    sys.exit(__doc__)
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

for c_mm in [float(a) for a in sys.argv[1:]]:
    c = c_mm / 1000.0
    hx, hy = (PEG_X + c) / 2, (PEG_Y + c) / 2      # half sizes of the hole
    zb = ZTOP_TABLE + FLOOR
    zt = zb + DEPTH
    zw = FLOOR + DEPTH / 2                          # center height of the walls (block frame)
    name = f'table_with_rectangular_hole_{c_mm:g}mm_clearance'
    xml = f'''<!--
  Environment for rectangular peg-in-hole insertion, for mujoco_robotsimulator_crospi.

  The peg is the rectangular tool of the robot (urdf/ur10e_rectangular_tool.urdf: cross-section {PEG_X*1000:.0f} mm (x of tool0)
  x {PEG_Y*1000:.0f} mm (y of tool0), 15 cm long, fixed to tool0). A steel block on the table has a rectangular hole with a
  DIAMETRAL clearance of {c_mm:g} mm in each direction:
      hole = ({PEG_X*1000:.0f} + {c_mm:g}) mm x ({PEG_Y*1000:.0f} + {c_mm:g}) mm = {2*hx*1000:.2f} mm x {2*hy*1000:.2f} mm   (clearance {c_mm/2:g} mm per side)
      hole depth = {DEPTH*1000:.0f} mm, flat bottom, sharp edges (no chamfer)
  World coordinates (with the identity base pose of the simulator configuration, world = robot base_link):
      hole axis: vertical, through ({CX}, {CY}); long side of the hole along the world x axis rotated by
      {YAW} rad about z (aligned with x of tool0 at MoveHome)
      hole entrance (block top surface): z = {zt:.3f}
      hole bottom: z = {zb:.3f}
      block: {2*B*100:.0f} x {2*B*100:.0f} x {(FLOOR+DEPTH)*100:.0f} cm on the table top (z = {ZTOP_TABLE})

  At the joints [4.7116, -1.5708, 2.0944, -2.0944, -1.5708, 0] (MoveHome) the peg is vertical, on the axis of the hole
  and aligned with it; its tip is {(0.241075 - zt)*1000:.0f} mm above the entrance. Full insertion (tip on the bottom)
  corresponds to tool0 at z = {zb + 0.15:.3f}.

  The hole is formed by 4 boxes around it (convex pieces: MuJoCo replaces concave meshes by their convex hull).
  This file was generated with generate_rectangular_peg_in_hole_environments.py (in the same directory).

  Sensors (published by the simulator on <topic_namespace>/mujoco_sensors/<name>):
    - block_top_touch:   normal contact force [N] on the top surface of the block (e.g. peg landing next to the hole)
    - hole_wall_touch:   normal contact force [N] on the walls of the hole
    - hole_bottom_touch: normal contact force [N] on the bottom of the hole
    - hole_entrance, hole_bottom: ground-truth positions of the center of the hole at the entrance and at the bottom [m]
    - hole_orientation: ground-truth orientation of the hole (quaternion w x y z of the hole frame: x along the long
      side, z up)

  The physics options (timestep, gravity, integrator, contacts) are set by the simulator configuration; use
  simulation_timestep <= 0.001 s (see the contact parameters below).
-->
<mujoco model="{name}">
  <compiler angle="radian"/>

  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.3 0.4 0.5" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="4 4" reflectance="0.1"/>
    <material name="wood" rgba="0.55 0.4 0.3 1"/>
    <material name="steel" rgba="0.6 0.62 0.65 1" specular="0.6" shininess="0.6"/>
  </asset>

  <default>
    <!-- Rigid, static contact surfaces for tight-tolerance insertion (same as table_with_hole_*mm_clearance.xml):
         - priority 1: these contact parameters are used (instead of mixing them with those of the peg);
         - solref timeconst 0.002 s, critically damped, and solimp 0.95-0.99: very stiff contact, penetration of
           a few micrometres at tens of N, i.e. much smaller than the clearance (>= 0.05 mm per side).
           REQUIRES simulation_timestep <= 0.001 s (MuJoCo limits the time constant to >= 2 x timestep).
           The high stiffness avoids that a peg landing on the sharp edge of the hole is artificially pushed into it;
         - friction: 0.3 (dry steel on steel/aluminium), small torsional and rolling friction;
         - condim 3: normal force + 2D tangential friction. -->
    <default class="rigid_surface">
      <geom priority="1" condim="3" friction="0.3 0.005 0.0001" solref="0.002 1" solimp="0.95 0.99 0.001 0.5 2"/>
    </default>
  </default>

  <worldbody>
    <light name="top" pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid"/>

    <!-- Table: 0.8 x 0.7 m, top surface at z = {ZTOP_TABLE} m -->
    <body name="table" pos="0.174 -0.65 {ZTOP_TABLE/2}">
      <geom name="table_top" class="rigid_surface" type="box" size="0.4 0.35 {ZTOP_TABLE/2}" material="wood"/>
    </body>

    <!-- Block with the hole. Body frame ('hole frame'): origin on the hole axis at the table top, x along the long side
         of the hole, z up. -->
    <body name="hole_block" pos="{CX} {CY} {ZTOP_TABLE}" euler="0 0 {YAW}">
      <!-- floor of the block (bottom of the hole at z = {FLOOR} in this frame). Separate (fixed) body, so that the
           touch sensor of the bottom does not count the contacts on the walls/top and vice versa. -->
      <body name="hole_floor">
        <geom name="block_floor" class="rigid_surface" type="box" pos="0 0 {FLOOR/2}" size="{B} {B} {FLOOR/2}" material="steel"/>
        <site name="hole_bottom_volume" type="box" pos="0 0 {FLOOR}" size="{hx:.6f} {hy:.6f} 0.001" group="4" rgba="0 1 0 0.2"/>
      </body>
      <!-- walls around the hole: +x / -x (short sides of the hole, full width of the block), +y / -y (long sides) -->
      <geom name="wall_px" class="rigid_surface" type="box" pos="{(hx + B)/2:.6f} 0 {zw}" size="{(B - hx)/2:.6f} {B} {DEPTH/2}" material="steel"/>
      <geom name="wall_nx" class="rigid_surface" type="box" pos="{-(hx + B)/2:.6f} 0 {zw}" size="{(B - hx)/2:.6f} {B} {DEPTH/2}" material="steel"/>
      <geom name="wall_py" class="rigid_surface" type="box" pos="0 {(hy + B)/2:.6f} {zw}" size="{hx:.6f} {(B - hy)/2:.6f} {DEPTH/2}" material="steel"/>
      <geom name="wall_ny" class="rigid_surface" type="box" pos="0 {-(hy + B)/2:.6f} {zw}" size="{hx:.6f} {(B - hy)/2:.6f} {DEPTH/2}" material="steel"/>
      <!-- Touch volumes (invisible, non-colliding). A MuJoCo touch sensor counts a contact of its body when the ray
           from the contact point along the contact normal crosses the site:
           - top: thin box just ABOVE the top surface, only reached by the vertical normals of contacts on the top;
           - walls: box filling the hole (0.01 mm smaller than the hole on each side), spanning the height of the walls:
             reached by the horizontal normals of contacts on the walls, not by the vertical normals of contacts on
             the rim (outside the hole). -->
      <site name="block_top_surface" type="box" pos="0 0 {FLOOR + DEPTH + 0.0015}" size="{B} {B} 0.001" group="4" rgba="0 1 0 0.2"/>
      <site name="hole_wall_volume" type="box" pos="0 0 {zw + 0.00015}" size="{hx - 0.00001:.6f} {hy - 0.00001:.6f} {DEPTH/2 + 0.00015:.5f}" group="4" rgba="0 1 0 0.2"/>
      <!-- Ground truth -->
      <site name="hole_entrance" type="sphere" pos="0 0 {FLOOR + DEPTH}" size="0.002" rgba="1 0.1 0.1 1"/>
      <site name="hole_bottom" type="sphere" pos="0 0 {FLOOR}" size="0.002" rgba="1 0.1 0.1 1"/>
    </body>
  </worldbody>

  <!-- MuJoCo sensors: published by the simulator on <topic_namespace>/mujoco_sensors/<name> -->
  <sensor>
    <touch name="block_top_touch" site="block_top_surface"/>
    <touch name="hole_wall_touch" site="hole_wall_volume"/>
    <touch name="hole_bottom_touch" site="hole_bottom_volume"/>
    <framepos name="hole_entrance" objtype="site" objname="hole_entrance"/>
    <framepos name="hole_bottom" objtype="site" objname="hole_bottom"/>
    <framequat name="hole_orientation" objtype="site" objname="hole_entrance"/>
  </sensor>
</mujoco>
'''
    open(os.path.join(OUT_DIR, f'{name}.xml'), 'w').write(xml)
    print(f'{name}.xml: hole {2*hx*1000:.2f} x {2*hy*1000:.2f} mm')
