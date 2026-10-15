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
ros2 run mujoco_robotsimulator_crospi run_crospi_tasks.py --task-list '$[betfsm_demos]/tasks/betfsm_demos_tasks.json' \
  --tasks MoveHome --repetitions 1 --tick-frequency 100 --publish-frequency 5
```

- Kill background `crospi_node`s by PID afterwards: `timeout ... ros2 run crospi_node` does NOT kill the node
  process, and two nodes with the same name make the BeTFSM tasks CANCEL. Check with `pgrep -af crospi_node`.
- The `sine` task of `betfsm_demos` runs forever (`execution_time: -10`); use `MoveHome` for pass/fail tests.
- `mujoco_viewer.py` opens a GUI window; test its data path headlessly by subscribing to the topics instead.

## Code map

- `src/mujoco_simulator_config.cpp`: parses and validates the JSON into `SimulatorConfig` (throws `std::runtime_error`).
- `src/mujoco_scene_builder.cpp`: URDF → `mjSpec`, attaches it to the environment MJCF (or empty world) through a
  frame at the base pose, adds ground plane, physics options, F/T sensors, contact excludes, compiles.
- `src/mujoco_robotsimulator_crospi.cpp`: the plugin. `update()` runs on crospi's driver thread every `periodicity`
  and performs all MuJoCo computation synchronously (`substeps = periodicity / simulation_timestep`).
- `src/visualization_publisher.cpp`: own rclcpp node + executor thread; model as MJB (transient local), state
  `[time, qpos...]`. The update thread only `try_lock`s a buffer (never blocks).
- `src/test_mujoco_robotsimulator.cpp`, `scripts/mujoco_viewer.py`, `scripts/run_crospi_tasks.py`: test executables.

## Project rules

- **No hidden parameters** (explicit user requirement). Every value affecting the simulation comes from the JSON
  config; executables take only required arguments. Do not add C++ defaults, argparse defaults or hard-coded tuning
  constants. Optional config keys may only enable optional features. Reject keys that do not apply to the selected
  `control_mode`.
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
- MuJoCo force sensor = wrench of parent on child subtree, in site frame (verified: hanging 2 kg → +19.62 N).
- `mjsJoint::damping` is an array in 3.16 (`damping[0]` is the linear coefficient).
- **crospi_node** constructs *all* entries of `robot/robotsimulators` (hence one setup file per mode), requires a
  top-level `"$defs": {}` and a `default_robot_specification`; it picks the plugin from the `is-<name>` key.
- `ROSRunner.run()` (betfsm_ros) re-parses `sys.argv`; strip the script's own arguments first. Pass `serve=False`
  to avoid its web server.
- `config/urdf/ur10e.urdf` was generated with xacro (not installed system-wide) from `ur_description`
  (`ur_type:=ur10e name:=ur10e`); `ur_description` must be built in this workspace for the meshes.
