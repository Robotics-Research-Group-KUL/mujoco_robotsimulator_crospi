# CLAUDE.md

Crospi `RobotSimulator` plugin (pluginlib) that simulates a robot loaded from a URDF with MuJoCo (C API, `mjSpec`).
User documentation is in [README.md](README.md); the parameter reference is the JSON schema
[json_schemas/mujoco_robotsimulator_crospi.schema.json](json_schemas/mujoco_robotsimulator_crospi.schema.json).
Crospi sources (read-only reference) are in `/opt/ros2_crospi_ws/src/crospi` (see its `AGENTS.md`).

## Build and test

Workspace root: `../..` (`crospi_ws`). ROS distro: kilted. Always `source install/setup.bash` after building.

```bash
cd ../.. && colcon build --symlink-install --packages-select mujoco_robotsimulator_crospi
```

There are no unit tests; verify changes by running the executables:

```bash
# plugin alone, all three modes (prints the max tracking error; kinematic ≈ 1e-5, servos < 0.07 rad)
for m in kinematic position_servo velocity_servo; do
  ros2 run mujoco_robotsimulator_crospi test_mujoco_robotsimulator \
    --config "\$[crospi_application_template]/applications/application_mujoco_examples_ur10e/ur10e_$m.setup.json" --simulator-index 0 \
    --duration 4 --amplitude 0.5 --frequency 0.25 --real-time false --csv none
done

# end-to-end: crospi_node + BeTFSM tasks (expect "Finished with outcome SUCCEED")
/opt/ros2_crospi_ws/install/crospi_core/lib/crospi_core/crospi_node --ros-args \
  -p config_file:='$[crospi_application_template]/applications/application_mujoco_examples_ur10e/ur10e_position_servo.setup.json' -p simulation:=true &
ros2 run mujoco_robotsimulator_crospi run_crospi_tasks.py --task-list '$[mujoco_robotsimulator_crospi]/betfsm_tasks_examples/betfsm_demos_tasks.json' \
  --tasks MoveHome --repetitions 1 --tick-frequency 100 --publish-frequency 5

# force-control demo, headless, recorded (expect "Runner finished with outcome SUCCEED"; ~70 s)
docs/record_demo.sh '$[crospi_application_template]/applications/application_mujoco_examples_ur10e/ur10e_contour_following_circular.setup.json' \
  ../crospi_application_template/skill_specifications/libraries/skill_lib_example/circular_contour_following_example/circular_contour_following_example.py \
  /tmp/circular.npz 240
```

- The example setup files and the tool URDFs live in `crospi_application_template`
  (`applications/application_mujoco_examples_ur10e/`, `robot_models/urdf_models/robot_setups/ur10e/`; copies of the
  URDFs stay in `config/urdf/`). The BeTFSM force-control demos (guarded motions, circular and large sine contour
  following) are in its `skill_specifications/libraries/skill_lib_example/` and documented in its README.
  `ur10e_position_servo.setup.json` uses the empty environment (`config/environments/empty/empty.xml`).
- Kill background `crospi_node`s by PID afterwards: `timeout ... ros2 run crospi_node` does NOT kill the node
  process, and two nodes with the same name make the BeTFSM tasks CANCEL. Check with `pgrep -af crospi_node`.
- `betfsm_tasks_examples/betfsm_demos_tasks.json` is a copy of the betfsm_demos task list with a modified `MoveHome`
  (robot above the table, so `sine` touches it). `sine` runs forever (`execution_time: -10`); use `MoveHome` for
  pass/fail tests. `MovingDown`/`MovingUp` need crospi_application_template.
- README.md is the user-facing reference (quick start, frames table, troubleshooting): keep it in sync.
- `mujoco_viewer.py` opens a GUI window; test its data path headlessly by subscribing to the topics instead.

## Code map

- `src/mujoco_simulator_config.cpp`: parses and validates the JSON into `SimulatorConfig` (throws `std::runtime_error`).
- `src/mujoco_scene_builder.cpp`: URDF → `mjSpec`, attaches it to the environment MJCF (or empty world) through a
  frame at the base pose, adds ground plane, physics options, contact excludes, compiles.
- `src/mujoco_robotsimulator_crospi.cpp`: the plugin. `update()` runs on crospi's driver thread every `periodicity`
  and performs all MuJoCo computation synchronously (`substeps = periodicity / simulation_timestep`).
- `src/ros_publisher.cpp`: the simulator's rclcpp node (`ros_node_name`, always created) + executor thread. Publishes
  the sensors (`sensor_publishing`, standard msgs, one timer) and optionally the visualization (model as MJB,
  transient local; state `[time, qpos...]`). The update thread passes `Measurements` + `sensordata` + `qpos` through
  a `try_lock`ed buffer (never blocks).
- External wrench (optional `external_wrench`): `RosPublisher` subscribes to `geometry_msgs/WrenchStamped` (frame_id =
  body name, wrench in that body frame at its origin); `apply_external_wrench()` writes `xfrc_applied` (world, torque
  about the COM) before the physics steps, clears it after `timeout`, clamps to `max_force`/`max_torque`; it is added
  to the TCP wrench when the body is on the end effector. `mujoco_viewer.py` publishes the mouse perturbation as an
  explicit spring (`--force-stiffness`, `--torque-stiffness`), NOT MuJoCo's `mjv_applyPerturbForce` (scaled by the
  body mass: ~0.6 N for a 2 cm drag of the 0.29 kg tool, useless against the servo).
- Contact forces (optional `visualization/contact_topic`): `compute_contacts()` fills a buffer preallocated to
  `max_contacts(m)` = `m->narena / sizeof(mjContact)` entries of `CONTACT_STRIDE` (8) values (no allocation in the
  update thread); published with the state. Force shown on the robot's moving bodies, else on the non-static body.
  The viewer draws them in `user_scn`; they point INTO that body (transparency, key T, needed to see them).
- `Measurements` (in `ros_publisher.hpp`) is computed once per update by `compute_measurements()` and used both by
  `write_feedback()` (crospi) and the publisher; the `need_*_` flags are the union of feedback and publication needs.
- `src/test_mujoco_robotsimulator.cpp`, `scripts/mujoco_viewer.py`, `scripts/run_crospi_tasks.py`: test executables.
- `docs/` (not installed): README images. `generate_snapshots.py` (environment PNGs); `record_demo.sh` (runs a
  crospi_node + BeTFSM script headless and records it with `record_states.py`, which stores the MJB model + states
  of the visualization topics in a `.npz`); `render_gif.py` (offscreen GIF from a `.npz`, free camera). The exact
  commands/cameras of the demo GIFs of crospi_application_template are in README "Documentation images and GIFs".

## Project rules

- **No hidden parameters** (explicit user requirement). Every value affecting the simulation comes from the JSON
  config; executables take only required arguments. Do not add C++ defaults, argparse defaults or hard-coded tuning
  constants. Optional config keys may only enable optional features. Reject keys that do not apply to the selected
  `control_mode`. Sole explicit exception (user request): omitting `sensor_publishing/sensors` publishes all
  available sensors.
- **Never put `default` in the JSON schema**: the crospi blackboard (`BlackBoard::process_and_validate`) inserts schema
  defaults for missing properties, which would silently enable features / hide missing parameters. Use `examples`.
- `control_mode` is an object with one `is-<mode>` key + that mode's parameters (crospi discriminator convention,
  schema `dependencies` keyed by `is-<mode>`); the old flat format (string `control_mode`, top-level gains) is rejected
  with a migration message.
- Keep the schema, `parse_config`, the README parameter table and the example setup files (in crospi_application_template) in sync.
- Follow the template conventions: namespace `etasl`, `PLUGINLIB_EXPORT_CLASS(..., etasl::RobotDriver)` while
  `plugins.xml` uses `base_class_type="etasl::RobotSimulator"`, errors built with `fmt::format`.

## Non-obvious facts (learned the hard way)

- **MuJoCo location**: no system install; CMake finds it through the pip wheel
  (`~/.local/lib/python3.12/site-packages/mujoco`, 3.16, `libmujoco.so.3.16.0` without unversioned symlink) unless
  `MUJOCO_DIR` or a CMake package is given. The rpath is embedded. The viewer's python `mujoco` must be the same
  version (MJB format is version specific).
- **Linking `crospi_core::crospi_utils`** (for `string_interpolate`/`JsonChecker`) needs the extra `find_package`
  calls listed in CMakeLists.txt (Eigen3, etasl, tf2, ...), otherwise CMake fails on its link interface.
- **URDF import**: set `spec->compiler.fusestatic = 0` (otherwise `tool0`/`flange` disappear) and
  `spec->strippath = 0`; mesh paths stay unresolved in the spec until compile, so `package://` is rewritten through
  the spec (`mjs_asMesh`). MuJoCo keeps only collision geometry (`discardvisual`); the UR visual `.dae` meshes share
  names with the collision meshes, so visual import is not offered.
- **Adjacent-link contacts**: MuJoCo does not filter parent/child contacts when the parent is welded to the world, so
  UR `shoulder_link` collides with `base_link_inertia` and joint 0 is blocked. The builder compiles once, then adds
  `mjs_addExclude` between adjacent robot weld groups and recompiles. Keep this.
- **`mj_saveXMLString` of an attached spec produces invalid XML** (empty default class); that is why the viewer gets
  the model as MJB binary.
- After `mj_step`, body poses/sensors correspond to the previous state: `step_dynamic()` calls `mj_forward` before
  reporting Cartesian/wrench/torque feedback.
- Torques are applied via `qfrc_applied` (no MuJoCo actuators); gravity compensation via body `gravcomp` (shows up in
  `qfrc_gravcomp`). Kinematic-mode torques come from `mj_inverse`.
- The TCP wrench is NOT a MuJoCo force sensor: on the UR, `tool0`/`flange` have no geometry (all of it is in
  `wrist_3_link`), so a sensor at `tool0` always read zero. `compute_tcp_wrench()` sums `mj_contactForce` over the
  contacts of the end-effector bodies (weld group of `tcp_link` + descendants), shifted to the tcp origin, sign =
  wrench the end effector exerts on the environment. `frame^T * f` = force of `geom[0]` on `geom[1]` (verified).
- `mjsJoint::damping` is an array in 3.16 (`damping[0]` is the linear coefficient).
- **crospi_node** constructs *all* entries of `robot/robotsimulators` (hence one setup file per mode), requires a
  top-level `"$defs": {}` and a `default_robot_specification`; it picks the plugin from the `is-<name>` key.
- `ros2` CLI tools print `RTPS_TRANSPORT_SHM ... open_and_lock_file failed` in this environment; harmless (stale
  Fast DDS shared-memory locks), messages still arrive.
- `ROSRunner.run()` (betfsm_ros) re-parses `sys.argv`; strip the script's own arguments first. Pass `serve=False`
  to avoid its web server.
- The example setups use `ur10e_cylinder_tool.urdf` (crospi_application_template; copy in `config/urdf/`) = `ur10e.urdf` + fixed links `cylinder_tool`
  (15 cm x Ø3 cm, after `tool0`) and `tool_tip`; `table_with_corner.xml` is dimensioned for this tool at `MoveHome`.
- `config/urdf/ur10e.urdf` was generated with xacro (not installed system-wide) from `ur_description`
  (`ur_type:=ur10e name:=ur10e`); `ur_description` must be built in this workspace for the meshes.

## Environment modelling (config/environments)

- Layout: one folder per task (`empty`, `pick_and_place`, `guarded_motions`, `contour_following`, `peg_insertion_cylindrical`,
  `peg_insertion_rectangular`); the peg-insertion generators live next to their files and write into their own folder.
  The whole `config/` directory is installed, so new folders need no CMake change.

- Concave shapes (holes, rings, sine walls) must be built from convex pieces (boxes or convex mesh prisms): MuJoCo
  replaces a concave mesh by its convex hull.
- MuJoCo `touch` sensors count a contact of the site's body when the RAY from the contact point along the contact
  normal crosses the site (not "contact point inside the site"). Separate surfaces by body (e.g. `hole_floor`) or by
  normal direction (e.g. thin cylinder on the hole axis for the wall, thin box just above the top surface).
- Sharp edges + soft contact: a peg landing on the rim of a hole was pushed sideways into the hole (overlap slivers
  at the edge are resolved horizontally). Peg-in-hole files use `solref 0.002` (needs timestep <= 1 ms) and the
  validation uses a guarded approach (2 cm/s); fast impacts can still show the artefact.
- Contour following: very stiff contacts (`solref 0.005`) chatter while sliding; contour files use `solref 0.02`.
- Peg-in-hole holes are placed at the EXACT peg axis and yaw at MoveHome from forward kinematics (tool0 at
  (0.173664, -0.614962, 0.391075), yaw of its x axis = pi - 0.000789 rad); rounding to (0.174, -0.615) puts the
  peg 0.34 mm off the axis, i.e. on the rim of the tight holes. Re-derive these constants if MoveHome or the
  URDF changes (the generators hold them as constants).
- README snapshots: `docs/images/*.png`, rendered by `docs/generate_snapshots.py` (needs MUJOCO_GL=glfw here: no EGL/OSMesa).
  Demo GIFs: `docs/record_demo.sh` + `docs/render_gif.py` (same MUJOCO_GL requirement; no ffmpeg here, GIFs are
  written with PIL). Look at sample frames before committing: from the snapshot camera of the large sine contour the
  wall hides the tool (use azimuth -160, elevation -55). The demo GIFs are copied into
  `crospi_application_template/docs/images/` (separate git repo, so no cross-repo image links).
- `pgrep -f crospi_node` also matches the shell running the command; use `pgrep -x` / `pkill -x crospi_node`.
  Re-render after changing an environment, and look at the images: they revealed that the round wall segments of the
  cylindrical-hole block protruded outside the square block.
- Validation scripts used so far (probe bodies on slide joints, IK reachability along the path) live in the session
  scratchpad, not in the repo; recreate them when changing the environments.
