# mujoco_robotsimulator_crospi Package

A [crospi](https://rob.pages.gitlab.kuleuven.be/crospi/) robot simulator plugin based on the
[MuJoCo](https://mujoco.org/) physics engine.

The robot is loaded from a **URDF** file, optionally placed in an **MJCF environment** (tables, objects, ...), and
simulated in MuJoCo. Every update of the crospi driver thread reads the joint velocity setpoints computed by eTaSL,
advances the simulation by `periodicity` seconds and writes the feedback (joint positions and, optionally, joint
velocities, joint torques, Cartesian pose/twist of a link, the wrench measured at a link, and the base pose/twist).

Three control modes are available:

| `control_mode`   | Simulation | Behaviour |
|------------------|------------|-----------|
| `kinematic`      | kinematic robot, dynamic environment | The joint positions are the integral of the setpoints (perfect tracking). The robot is not stopped by contacts, but it pushes the objects of the environment. Equivalent to `simple_kinematic_simulation`, plus MuJoCo kinematics, contacts and visualization. |
| `position_servo` | dynamic | A joint PD controller tracks the integrated setpoint: `tau = joint_stiffness (q_ref - q) + joint_damping_gains (qd_sp - qd)`. Behaves like the internal position controller of an industrial robot (tracking errors, contacts, torque limits). |
| `velocity_servo` | dynamic | A joint velocity controller tracks the setpoint: `tau = joint_damping_gains (qd_sp - qd)`. |

In the dynamic modes, the weight of the robot can be compensated (`gravity_compensation`) and the motor torques can be
limited to the effort limits of the URDF (`clamp_torques_to_effort_limits`).

## Contents

| Path | Description |
|------|-------------|
| `src/mujoco_robotsimulator_crospi.cpp` | The plugin (`etasl::mujoco_robotsimulator_crospi`, base class `etasl::RobotSimulator`) |
| `src/mujoco_simulator_config.cpp` | Reading and validation of the JSON configuration |
| `src/mujoco_scene_builder.cpp` | URDF + environment → MuJoCo model |
| `src/visualization_publisher.cpp` | Optional publication of the model and state for the viewer |
| `json_schemas/mujoco_robotsimulator_crospi.schema.json` | JSON schema of the configuration (all parameters are documented there) |
| `src/test_mujoco_robotsimulator.cpp` | Executable: tests the plugin without the crospi_node |
| `scripts/mujoco_viewer.py` | Executable: MuJoCo viewer that mirrors the running simulation |
| `scripts/run_crospi_tasks.py` | Executable: end-to-end test, runs crospi tasks on a crospi_node that uses the simulator |
| `config/urdf/ur10e.urdf` | UR10e URDF (generated from `ur_description`, meshes from `package://ur_description`) |
| `config/environments/table_with_box.xml` | Example MJCF environment |
| `config/ur10e_{kinematic,position_servo,velocity_servo}.setup.json` | Example crospi setup files, one per control mode |

## Dependencies

- crospi (`crospi_core`) and its dependencies (ROS 2 Humble or newer, Boost 1.89).
- **MuJoCo ≥ 3.3** (C API with `mjSpec`). It is not available as a rosdep key; at build time it is located, in this
  order, as:
  1. a CMake package (`find_package(mujoco CONFIG)`, e.g. MuJoCo built from source and installed);
  2. an explicit directory given with `-DMUJOCO_DIR=<dir>` or the environment variable `MUJOCO_DIR` (release tarball
     layout `include/` + `lib/`, or the pip wheel layout);
  3. the MuJoCo python package (`pip install mujoco`), which ships the headers and `libmujoco.so`.

  The path of the library is embedded in the run-time path of the plugin.
- `ur_description` (Universal_Robots_ROS2_Description) for the meshes of the example UR10e.
- For the viewer: the MuJoCo python package (`pip install mujoco`), **with the same version as the MuJoCo library
  the plugin was built with** (the binary model format is version specific).
- For `run_crospi_tasks.py`: `betfsm`, `betfsm_ros`, `betfsm_crospi` and `betfsm_demos` (example task list).

## Building

```bash
cd <your_ws>
colcon build --symlink-install --packages-up-to mujoco_robotsimulator_crospi
source install/setup.bash
```

The CMake output reports which MuJoCo installation was found.

## Configuration

The simulator is configured as an entry of `robot/robotsimulators` in the crospi setup file and is selected with
`"is-mujoco_robotsimulator_crospi": true`. The complete documentation of every parameter is in the
[JSON schema](json_schemas/mujoco_robotsimulator_crospi.schema.json); a complete example is
[config/ur10e_position_servo.setup.json](config/ur10e_position_servo.setup.json):

```json
{
    "is-mujoco_robotsimulator_crospi": true,
    "robot_joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
    "initial_joints": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
    "periodicity": 0.01,
    "urdf_path": "$[mujoco_robotsimulator_crospi]/config/urdf/ur10e.urdf",
    "environment_mjcf_path": "$[mujoco_robotsimulator_crospi]/config/environments/table_with_box.xml",
    "base_position": [0.0, 0.0, 0.0],
    "base_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    "add_ground_plane": false,
    "simulation_timestep": 0.001,
    "integrator": "implicitfast",
    "gravity": [0.0, 0.0, -9.81],
    "contacts_enabled": true,
    "joint_armature": [0.5, 0.5, 0.3, 0.1, 0.1, 0.1],
    "joint_damping": [1.0, 1.0, 1.0, 0.5, 0.5, 0.5],
    "control_mode": "position_servo",
    "joint_stiffness": [8000.0, 8000.0, 5000.0, 1000.0, 1000.0, 1000.0],
    "joint_damping_gains": [300.0, 300.0, 150.0, 30.0, 30.0, 30.0],
    "gravity_compensation": true,
    "clamp_torques_to_effort_limits": true,
    "base_link": "base_link",
    "tcp_link": "tool0",
    "name_expr_cartesian_pos": "tcp_pos",
    "name_expr_cartesian_wrench": "tcp_wrench",
    "visualization": {
        "node_name": "mujoco_robotsimulator",
        "model_topic": "/mujoco_sim/model",
        "state_topic": "/mujoco_sim/state",
        "publish_period": 0.033
    }
}
```

### No hidden parameters

Every parameter that influences the simulation is read from the configuration; the plugin has no internal default
values. Required parameters must be present (otherwise the simulator throws an exception with a descriptive message),
and optional parameters only enable optional features (`environment_mjcf_path`, `joint_armature`/`joint_damping`/
`joint_frictionloss` overrides, `name_expr_*` feedback, `visualization`). Parameters that do not apply to the selected
`control_mode` are rejected.

The schema intentionally contains no `default` values, because the crospi blackboard inserts schema defaults for
missing properties, which would silently enable features or hide missing parameters. It provides `examples` instead
(used by the IDE autocompletion).

### Summary of the parameters

| Parameter | Required | Description |
|-----------|----------|-------------|
| `robot_joints`, `initial_joints`, `periodicity` | yes | Same meaning as for any crospi simulator. `periodicity` must be an integer multiple of `simulation_timestep`. |
| `urdf_path` | yes | URDF of the robot (`$[pkg]/...`, `package://pkg/...`, `file://`, absolute). |
| `environment_mjcf_path` | no | MJCF scene in which the robot is placed. |
| `base_position`, `base_quaternion_xyzw` | yes | Pose of the URDF root link in the world. |
| `add_ground_plane` | yes | Adds an infinite plane at z = 0. |
| `simulation_timestep`, `integrator`, `gravity`, `contacts_enabled` | yes | MuJoCo physics options (they overwrite those of the environment MJCF). |
| `joint_armature`, `joint_damping`, `joint_frictionloss` | no | Per-joint overrides of the URDF joint dynamics. |
| `control_mode` | yes | `kinematic`, `position_servo` or `velocity_servo`. |
| `joint_stiffness` | `position_servo` | Kp of the joint controller. |
| `joint_damping_gains` | `position_servo`, `velocity_servo` | Kd of the joint controller. |
| `gravity_compensation`, `clamp_torques_to_effort_limits` | `position_servo`, `velocity_servo` | See above. |
| `base_link`, `tcp_link` | if Cartesian/base feedback is requested | Links used for the Cartesian and base feedback. |
| `name_expr_*` | no | Enable the corresponding feedback (as in `simple_kinematic_simulation`). `name_expr_joint_current` is not supported. |
| `visualization` | no | Publishes the model and the state for `mujoco_viewer.py`. |

### Feedback conventions

- `joint_vel`: simulated joint velocities.
- `joint_torque`: in the dynamic modes, the motor torque (controller + gravity compensation); in the kinematic mode,
  the inverse-dynamics torque of the commanded motion.
- `cartesian_pos` / `cartesian_quat`: pose of `tcp_link` with respect to `base_link`.
- `cartesian_twist`: twist of `tcp_link` relative to `base_link`, reference point at the origin of `tcp_link`, expressed
  in `base_link`.
- `cartesian_wrench`: simulated force/torque sensor at the origin of `tcp_link`, expressed in `tcp_link`. It reports
  the wrench exerted by the parent link on `tcp_link` and its child links. For a quasi-static tool of negligible mass,
  this is the wrench that the tool exerts on the environment; it includes the weight of the links below the sensor, as
  a real force/torque sensor does. Only contacts on `tcp_link` and its child links are measured.
- `base_pos` / `base_quat` / `base_twist`: pose and twist (reference point at its origin, expressed in the world) of
  `base_link` in the world of the simulation.

### URDF to MuJoCo conversion

- The URDF is parsed by MuJoCo. Mesh paths (`package://`, `file://`, absolute or relative to the URDF) are resolved by
  the plugin. Xacro files are not supported: generate the URDF first (`xacro robot.urdf.xacro > robot.urdf`).
- Links attached with fixed joints (e.g. `tool0`, `flange`) are kept as separate bodies, so they can be used as
  `base_link`/`tcp_link`.
- MuJoCo only imports the **collision** geometry of the URDF, which is used both for the simulation and the
  visualization. Supported mesh formats are STL, OBJ and MSH (not DAE).
- Contacts between adjacent links of the robot (consecutive links in the URDF tree) are excluded, as in other
  simulators. Without this, the first moving link of a robot fixed to the world would collide with the base link,
  because MuJoCo does not filter contacts with bodies welded to the world.
- URDF joints that are not listed in `robot_joints` are simulated as passive joints.
- The URDF effort limits (`<limit effort>`) are used by `clamp_torques_to_effort_limits`; the URDF joint limits are
  enforced by MuJoCo in the dynamic modes. In the kinematic mode, the robot follows the setpoints exactly.
- When the controller is activated (`on_activate`), the position reference of the `position_servo` is reset to the
  current joint positions to avoid wind-up.
- In the dynamic modes, the stability depends on `simulation_timestep`, the gains and the joint inertia. Adding
  `joint_armature` (reflected rotor inertia) greatly improves the stability of stiff controllers.
- The contacts present at the initial configuration are printed when the simulator is initialized, which helps to
  detect an initial configuration in collision.

## Executables

All executables require all their arguments (there are no hidden parameters).

### 1. `test_mujoco_robotsimulator`: test the plugin without crospi_node

Loads a simulator from a crospi setup file through pluginlib (exactly as the crospi_node does), drives all joints with
`qd(t) = amplitude sin(2π frequency t)` and reports the tracking error with respect to the ideal integrated motion.

```bash
ros2 run mujoco_robotsimulator_crospi test_mujoco_robotsimulator \
    --config "\$[mujoco_robotsimulator_crospi]/config/ur10e_position_servo.setup.json" \
    --simulator-index 0 --duration 10 --amplitude 0.5 --frequency 0.25 \
    --real-time true --csv /tmp/mujoco_test.csv
```

- `--simulator-index`: entry of `blackboard/robot/robotsimulators`.
- `--real-time false` runs as fast as possible (the real-time factor is reported).
- `--csv none` disables the CSV output (time, setpoints, ideal and simulated positions, torques and wrench if
  available).

It works with any crospi simulator plugin (e.g. `simple_kinematic_simulation`).

### 2. `mujoco_viewer.py`: visualize the simulation

If the simulator configuration contains a `visualization` object, the simulator publishes the compiled model
(`std_msgs/UInt8MultiArray`, transient local, MJB format) and the state `[time, qpos...]`
(`std_msgs/Float64MultiArray`). The viewer receives the model, opens the MuJoCo viewer and mirrors the state (it does
not simulate):

```bash
ros2 run mujoco_robotsimulator_crospi mujoco_viewer.py \
    --model-topic /mujoco_sim/model --state-topic /mujoco_sim/state --render-rate 60
```

The viewer can be started before or after the simulator, and reloads the model when the simulator is restarted.

### 3. `run_crospi_tasks.py`: end-to-end test with the crospi_node

Runs a sequence of crospi tasks (with BeTFSM) on a crospi_node that uses the MuJoCo simulator. In separate terminals:

```bash
# 1. crospi node with the MuJoCo simulator (also kinematic / velocity_servo setup files available)
ros2 run crospi_core crospi_node --ros-args \
    -p config_file:="\$[mujoco_robotsimulator_crospi]/config/ur10e_position_servo.setup.json" -p simulation:=true

# 2. viewer (optional)
ros2 run mujoco_robotsimulator_crospi mujoco_viewer.py \
    --model-topic /mujoco_sim/model --state-topic /mujoco_sim/state --render-rate 60

# 3. tasks: move home in joint space, then a Cartesian sine wave (runs until Ctrl-C)
ros2 run mujoco_robotsimulator_crospi run_crospi_tasks.py \
    --task-list "\$[betfsm_demos]/tasks/betfsm_demos_tasks.json" \
    --tasks MoveHome sine --repetitions 1 --tick-frequency 100 --publish-frequency 5
```

The example setup files contain a `default_robot_specification` with the same URDF, a `JointStateOutputHandler`
(`/joint_states`) and `"$defs": {}` (required by the crospi_node). They do not reference a bundled
`"$schema"`; when used inside an application package (e.g. `crospi_application_template`), add the
schema of this package to the bundled schema of the application.

## Documentation

For documentation and tutorials, checkout our [crospi website](https://rob.pages.gitlab.kuleuven.be/crospi/), in
particular the tutorials on
[robot simulators](https://rob.pages.gitlab.kuleuven.be/crospi/markdown/advanced_tutorials/simulators/) and the
[setup configuration](https://rob.pages.gitlab.kuleuven.be/crospi/markdown/basic_tutorials/setup_configuration/).


## License and acknowledgements

Published under the [GNU LESSER GENERAL PUBLIC LICENSE](LICENSE) Version 3, 29 June 2007.


<a href="https://aiprism.eu/"><img src="./Ai-Prism_Logo_Horizontal.png" alt="AI-PRISM Logo" width="150" /></a>
This work was funded by the European Union’s Horizon 2020 research and innovation program
under the grant agreement No. <a href="https://cordis.europa.eu/project/id/101058589">101058589</a> ( <a href="https://aiprism.eu/">AI-Prism</a>)

<p/>

<p/>

<p/>

## Authors
<p float="left">
<a href="https://www.kuleuven.be/english/kuleuven/">
    <img src="./logo_kuleuven.png" alt="KU Leuven Logo" width="150"/>
</a>
<a href="https://www.mech.kuleuven.be/en/research/ram">
    <img src="./logo_RAM.png" alt="RAM Logo" width="110" />
</a>
</p>

(c) 2026, KU Leuven, Department of Mechanical Engineering, ROB-Group:

- [Santiago Iregui Rincon](https://www.kuleuven.be/wieiswie/en/person/00125886)
