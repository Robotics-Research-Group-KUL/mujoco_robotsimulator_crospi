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
Renders an animated GIF from a recording of record_states.py (offscreen, fixed camera, contact tool in orange as in
the snapshots of generate_snapshots.py). The recorded states are sampled every speedup / fps seconds of simulation
time (nearest recorded state). The python mujoco must have the same version as the simulator (MJB format).

Usage (needs an OpenGL context, e.g. MUJOCO_GL=glfw):
    python3 docs/render_gif.py --input rec.npz --output demo.gif --t-start 0 --t-end 0 --speedup 3 --fps 12 \
        --width 640 --height 360 --lookat 0.12 -0.55 0.25 --distance 1.5 --azimuth 140 --elevation -32

All arguments are required:
  --input, --output             recording (.npz) and GIF
  --t-start, --t-end            simulation time interval [s] (clipped to the recording; --t-end 0: until the end)
  --speedup, --fps              playback speed w.r.t. real time and frames per second of the GIF
  --width, --height             image size [px]
  --lookat, --distance, --azimuth, --elevation   free camera (as the MuJoCo viewer: [m], [m], [deg], [deg])
"""

import argparse
import os
import tempfile

import mujoco
import numpy as np
from PIL import Image

TOOL_BODIES = ("cylinder_tool", "rectangular_tool")
TOOL_RGBA = [0.95, 0.55, 0.1, 1.0]
GIF_COLORS = 128


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--t-start", required=True, type=float)
    parser.add_argument("--t-end", required=True, type=float)
    parser.add_argument("--speedup", required=True, type=float)
    parser.add_argument("--fps", required=True, type=float)
    parser.add_argument("--width", required=True, type=int)
    parser.add_argument("--height", required=True, type=int)
    parser.add_argument("--lookat", required=True, type=float, nargs=3)
    parser.add_argument("--distance", required=True, type=float)
    parser.add_argument("--azimuth", required=True, type=float)
    parser.add_argument("--elevation", required=True, type=float)
    args = parser.parse_args()

    rec = np.load(args.input)
    with tempfile.NamedTemporaryFile(suffix=".mjb", delete=False) as f:
        f.write(rec["mjb"].tobytes())
    try:
        m = mujoco.MjModel.from_binary_path(f.name)
    finally:
        os.remove(f.name)
    m.vis.global_.offwidth, m.vis.global_.offheight = args.width, args.height
    for name in TOOL_BODIES:
        body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        if body >= 0:
            m.geom_rgba[m.geom_bodyid == body] = TOOL_RGBA
    d = mujoco.MjData(m)

    states = rec["states"]
    times = states[:, 0]
    t0 = max(args.t_start, times[0])
    t1 = min(args.t_end, times[-1]) if args.t_end > 0 else times[-1]

    cam = mujoco.MjvCamera()
    cam.lookat[:] = args.lookat
    cam.distance, cam.azimuth, cam.elevation = args.distance, args.azimuth, args.elevation
    frames = []
    with mujoco.Renderer(m, args.height, args.width) as renderer:
        for t in np.arange(t0, t1, args.speedup / args.fps):
            d.qpos[:] = states[min(np.searchsorted(times, t), len(times) - 1), 1:]
            mujoco.mj_forward(m, d)
            renderer.update_scene(d, camera=cam)
            frames.append(Image.fromarray(renderer.render()).quantize(colors=GIF_COLORS,
                                                                       method=Image.Quantize.MEDIANCUT))
    frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=int(1000 / args.fps), loop=0,
                   optimize=True)
    print(f"Saved {args.output}: {len(frames)} frames, simulation time {t0:.1f}-{t1:.1f} s, "
          f"{os.path.getsize(args.output) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
