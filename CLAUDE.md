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
    --config "\$[mujoco_robotsimulator_crospi]/config/ur10e_$m.setup.json" --simulator-index 0 \
    --duration 4 --amplitude 0.5 --frequency 0.25 --real-time false --csv none
done

# end-to-end: crospi_node + BeTFSM tasks (expect "Finished with outcome SUCCEED")
/opt/ros2_crospi_ws/install/crospi_core/lib/crospi_core/crospi_node --ros-args \
  -p config_file:='$[mujoco_robotsimulator_crospi]/config/ur10e_position_servo.setup.json' -p simulation:=true &
ros2 run mujoco_robotsimulator_crospi run_crospi_tasks.py --task-list '$[mujoco_robotsimulator_crospi]/betfsm_tasks_examples/betfsm_demos_tasks.json' \
  --tasks MoveHome --repetitions 1 --tick-frequency 100 --publish-frequency 5
```

- Kill background `crospi_node`s by PID afterwards: `timeout ... ros2 run crospi_node` does NOT kill the node
  process, and two nodes with the same name make the BeTFSM tasks CANCEL. Check with `pgrep -af crospi_node`.
- `betfsm_tasks_examples/betfsm_demos_tasks.json` is a copy of the betfsm_demos task list with a modified `MoveHome`
  (robot above the table, so `sine` touches it). `sine` runs forever (`execution_time: -10`); use `MoveHome` for
  pass/fail tests. `MovingDown`/`MovingUp` need crospi_application_template (not installed).
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
- `Measurements` (in `ros_publisher.hpp`) is computed once per update by `compute_measurements()` and used both by
  `write_feedback()` (crospi) and the publisher; the `need_*_` flags are the union of feedback and publication needs.
- `src/test_mujoco_robotsimulator.cpp`, `scripts/mujoco_viewer.py`, `scripts/run_crospi_tasks.py`: test executables.

## Project rules

- **No hidden parameters** (explicit user requirement). Every value affecting the simulation comes from the JSON
  config; executables take only required arguments. Do not add C++ defaults, argparse defaults or hard-coded tuning
  constants. Optional config keys may only enable optional features. Reject keys that do not apply to the selected
  `control_mode`. Sole explicit exception (user request): omitting `sensor_publishing/sensors` publishes all
  available sensors.
- **Never put `default` in the JSON schema**: the crospi blackboard (`BlackBoard::process_and_validate`) inserts schema
  defaults for missing properties, which would silently enable features / hide missing parameters. Use `examples`.
- Keep the schema, `parse_config`, the README parameter table and the example `config/*.setup.json` in sync.
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
- The example setups use `config/urdf/ur10e_cylinder_tool.urdf` = `ur10e.urdf` + fixed links `cylinder_tool`
  (15 cm x Ø3 cm, after `tool0`) and `tool_tip`; `table_with_corner.xml` is dimensioned for this tool at `MoveHome`.
- `config/urdf/ur10e.urdf` was generated with xacro (not installed system-wide) from `ur_description`
  (`ur_type:=ur10e name:=ur10e`); `ur_description` must be built in this workspace for the meshes.

## Environment modelling (config/environments)

- Concave shapes (holes, rings, sine walls) must be built from convex pieces (boxes or convex mesh prisms): MuJoCo
  replaces a concave mesh by its convex hull.
- MuJoCo `touch` sensors count a contact of the site's body when the RAY from the contact point along the contact
  normal crosses the site (not "contact point inside the site"). Separate surfaces by body (e.g. `hole_floor`) or by
  normal direction (e.g. thin cylinder on the hole axis for the wall, thin box just above the top surface).
- Sharp edges + soft contact: a peg landing on the rim of a hole was pushed sideways into the hole (overlap slivers
  at the edge are resolved horizontally). Peg-in-hole files use `solref 0.002` (needs timestep <= 1 ms) and the
  validation uses a guarded approach (2 cm/s); fast impacts can still show the artefact.
- Contour following: very stiff contacts (`solref 0.005`) chatter while sliding; contour files use `solref 0.02`.
- Validation scripts used so far (probe bodies on slide joints, IK reachability along the path) live in the session
  scratchpad, not in the repo; recreate them when changing the environments.
