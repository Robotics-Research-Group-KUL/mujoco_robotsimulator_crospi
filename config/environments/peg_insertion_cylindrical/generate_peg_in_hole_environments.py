#!/usr/bin/env python3
"""
Generates the peg-in-hole environments table_with_hole_<c>mm_clearance.xml (in the directory of this script).

The peg is the cylindrical tool of urdf/ur10e_cylinder_tool.urdf (diameter 30 mm). The clearance is DIAMETRAL:
hole diameter = peg diameter + clearance (radial clearance = clearance / 2 per side).

Usage: python3 generate_peg_in_hole_environments.py <clearance_mm> [<clearance_mm> ...]
       e.g. python3 generate_peg_in_hole_environments.py 0.1 0.5 1 2
"""
import math
import os
import sys
PEG_D = 0.030                      # diameter of the cylindrical tool (peg)
N = 180                            # segments of the hole wall
B = 0.05                           # half side of the square block
DEPTH = 0.05; FLOOR = 0.01         # hole depth, thickness under the hole
cx, cy, ztop_table = 0.173664, -0.614962, 0.15   # axis of the peg at MoveHome (forward kinematics of the UR10e)
if len(sys.argv) < 2:
    sys.exit(__doc__)
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
for c_mm in [float(a) for a in sys.argv[1:]]:
    c = c_mm/1000.0
    R = (PEG_D + c)/2               # hole radius (apothem of the polygonal wall)
    A = 0.03                         # half side of the square opening of the outer frame of the block
    R_OUT = A*math.sqrt(2) + 0.0015  # outer apothem of the ring segments: covers the corners of the square opening
    T = R_OUT - R                    # radial thickness of the ring segments (they stay inside the block: R_OUT < B)
    hl = (R + T)*math.tan(math.pi/N)
    zb = ztop_table + FLOOR          # hole bottom
    zt = zb + DEPTH                  # block top / hole entrance
    dev = R*(1/math.cos(math.pi/N) - 1)*1000
    segs = '\n'.join(
        f'        <geom name="wall_{i:03d}" class="rigid_surface" type="box" '
        f'pos="{(R+T/2)*math.cos(2*math.pi*i/N):.7f} {(R+T/2)*math.sin(2*math.pi*i/N):.7f} {FLOOR + DEPTH/2}" '
        f'euler="0 0 {2*math.pi*i/N:.7f}" size="{T/2:.7f} {hl:.7f} {DEPTH/2}" material="steel"/>' for i in range(N))
    name = f'table_with_hole_{c_mm:g}mm_clearance'
    xml = f'''<!--
  Environment for peg-in-hole insertion, for mujoco_robotsimulator_crospi.

  The peg is the cylindrical contact tool of the robot (urdf/ur10e_cylinder_tool.urdf: diameter {PEG_D*1000:.0f} mm, 15 cm long,
  fixed to tool0). A steel block on the table has a cylindrical hole with a DIAMETRAL clearance of {c_mm:g} mm:
      hole diameter = {PEG_D*1000:.0f} mm + {c_mm:g} mm = {2*R*1000:.1f} mm   (radial clearance {c_mm/2:g} mm per side)
      hole depth    = {DEPTH*1000:.0f} mm, flat bottom, no chamfer
  World coordinates (with the identity base pose of the simulator configuration, world = robot base_link):
      hole axis: vertical, through ({cx}, {cy})
      hole entrance (block top surface): z = {zt:.3f}
      hole bottom: z = {zb:.3f}
      block: {2*B*100:.0f} x {2*B*100:.0f} x {(FLOOR+DEPTH)*100:.0f} cm on the table top (z = {ztop_table})

  At the joints [4.7116, -1.5708, 2.0944, -2.0944, -1.5708, 0] (MoveHome) the peg is vertical and its tip is at
  ({cx}, {cy}, 0.241): exactly on the axis of the hole, {(0.241-zt)*1000:.0f} mm above the entrance, so that a straight
  vertical motion from MoveHome inserts the peg. Full insertion (tip on the
  bottom) corresponds to tool0 at z = {zb + 0.15:.3f}.

  The hole wall is made of {N} box segments (MuJoCo replaces concave meshes by their convex hull, so a single mesh with
  a hole cannot be used). Their inner faces form a regular {N}-gon with apothem {R*1000:.2f} mm: the deviation from the
  circle is at most {dev:.4f} mm, much smaller than the clearance.
  This file was generated with generate_peg_in_hole_environments.py (in the same directory).

  Sensors (published by the simulator on <topic_namespace>/mujoco_sensors/<name>):
    - block_top_touch:   normal contact force [N] on the top surface of the block (e.g. peg landing next to the hole)
    - hole_wall_touch:   normal contact force [N] on the wall of the hole
    - hole_bottom_touch: normal contact force [N] on the bottom of the hole
    - hole_entrance, hole_bottom: ground-truth positions of the center of the hole at the entrance and at the bottom [m]

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
    <!-- Rigid, static contact surfaces for tight-tolerance insertion:
         - priority 1: these contact parameters are used (instead of mixing them with those of the peg);
         - solref timeconst 0.002 s, critically damped, and solimp 0.95-0.99: very stiff contact, penetration of
           a few micrometres at tens of N, i.e. much smaller than the radial clearance (>= 0.05 mm).
           REQUIRES simulation_timestep <= 0.001 s (MuJoCo limits the time constant to >= 2 x timestep).
           The high stiffness is needed at the sharp edge of the hole: with a softer contact, the peg sinks into the
           rim during landing deeper than the width of the thin overlaps at the edge, and MuJoCo resolves these
           overlaps sideways, which artificially pushes a peg resting on the rim into the hole;
         - friction: 0.3 (dry steel on steel/aluminium), small torsional and rolling friction;
         - condim 3: normal force + 2D tangential friction. -->
    <default class="rigid_surface">
      <geom priority="1" condim="3" friction="0.3 0.005 0.0001" solref="0.002 1" solimp="0.95 0.99 0.001 0.5 2"/>
    </default>
  </default>

  <worldbody>
    <light name="top" pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid"/>

    <!-- Table: 0.8 x 0.7 m, top surface at z = {ztop_table} m -->
    <body name="table" pos="0.174 -0.65 {ztop_table/2}">
      <geom name="table_top" class="rigid_surface" type="box" size="0.4 0.35 {ztop_table/2}" material="wood"/>
    </body>

    <!-- Block with the hole. Body frame: origin at the hole axis on the table top ({cx}, {cy}, {ztop_table}). -->
    <body name="hole_block" pos="{cx} {cy} {ztop_table}">
      <!-- floor of the block (bottom of the hole at z = {FLOOR} in this frame). Separate (fixed) body, so that the
           touch sensor of the bottom does not count the contacts on the wall/top and vice versa. -->
      <body name="hole_floor">
        <geom name="block_floor" class="rigid_surface" type="box" pos="0 0 {FLOOR/2}" size="{B} {B} {FLOOR/2}" material="steel"/>
        <site name="hole_bottom_volume" type="cylinder" pos="0 0 {FLOOR}" size="{R:.5f} 0.001" group="4" rgba="0 1 0 0.2"/>
      </body>
      <!-- outer square frame of the block: square opening of half side {A}, filled by the ring segments below -->
      <geom name="block_side_px" class="rigid_surface" type="box" pos="{(A + B)/2} 0 {FLOOR + DEPTH/2}" size="{(B - A)/2} {B} {DEPTH/2}" material="steel"/>
      <geom name="block_side_nx" class="rigid_surface" type="box" pos="{-(A + B)/2} 0 {FLOOR + DEPTH/2}" size="{(B - A)/2} {B} {DEPTH/2}" material="steel"/>
      <geom name="block_side_py" class="rigid_surface" type="box" pos="0 {(A + B)/2} {FLOOR + DEPTH/2}" size="{A} {(B - A)/2} {DEPTH/2}" material="steel"/>
      <geom name="block_side_ny" class="rigid_surface" type="box" pos="0 {-(A + B)/2} {FLOOR + DEPTH/2}" size="{A} {(B - A)/2} {DEPTH/2}" material="steel"/>
      <!-- wall of the hole: {N} segments, inner apothem {R*1000:.2f} mm -->
{segs}
      <!-- Touch volumes (invisible, non-colliding). A MuJoCo touch sensor counts a contact of its body when the ray
           from the contact point along the contact normal crosses the site:
           - top: thin box just ABOVE the top surface, only reached by the vertical normals of contacts on the top;
           - wall: thin cylinder ON THE AXIS of the hole, spanning the height of the wall: only reached by the
             horizontal (radial) normals of contacts on the wall, not by the vertical normals of contacts on the rim. -->
      <site name="block_top_surface" type="box" pos="0 0 {FLOOR + DEPTH + 0.0015}" size="{B} {B} 0.001" group="4" rgba="0 1 0 0.2"/>
      <site name="hole_wall_volume" type="cylinder" pos="0 0 {FLOOR + DEPTH/2 + 0.00015}" size="0.005 {DEPTH/2 + 0.00015:.5f}" group="4" rgba="0 1 0 0.2"/>
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
  </sensor>
</mujoco>
'''
    open(os.path.join(OUT_DIR, f'{name}.xml'), 'w').write(xml)
    print(f'{name}.xml: hole diameter {2*R*1000:.2f} mm, polygon deviation {dev:.4f} mm')
