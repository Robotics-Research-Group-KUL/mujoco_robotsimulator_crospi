//  Copyright (c) 2026 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <jsoncpp/json/json.h>
#include "crospi_utils/json_checker.hpp"

namespace etasl {
namespace mujoco_sim {

/**
 * @brief How the joint velocity setpoints coming from crospi are applied to the simulated robot.
 */
enum class ControlMode {
    KINEMATIC,       ///< Joint positions are integrated from the setpoints (perfect tracking). The rest of the scene is simulated dynamically.
    POSITION_SERVO,  ///< Dynamic simulation. A joint PD controller tracks the integrated setpoint: tau = Kp (q_ref - q) + Kd (qd_sp - qd)
    VELOCITY_SERVO   ///< Dynamic simulation. A joint velocity controller tracks the setpoint: tau = Kd (qd_sp - qd)
};

/**
 * @brief Optional publication of the simulation for visualization (see scripts/mujoco_viewer.py).
 */
struct VisualizationConfig {
    std::string model_topic;   ///< Topic (transient local) on which the compiled model is published once as MJB binary
    std::string state_topic;   ///< Topic on which [time, qpos...] is published
    double publish_period;     ///< Period in seconds of the state publication
};

/**
 * @brief Optional input of an external wrench applied to a body of the simulation (e.g. the mouse perturbations of
 *        scripts/mujoco_viewer.py, a joystick, ...), received as geometry_msgs/WrenchStamped on a ROS 2 topic.
 */
struct ExternalWrenchConfig {
    std::string topic;         ///< Topic of the geometry_msgs/WrenchStamped messages
    double timeout;            ///< The wrench is removed if no message is received during this time [s]
    double max_force;          ///< The force is scaled down to this norm if larger [N]
    double max_torque;         ///< The torque is scaled down to this norm if larger [Nm]
};

/** Names of the built-in sensors that can be published (besides the sensors defined in the MJCF environment). */
inline const char* SENSOR_JOINT_STATE = "joint_state";
inline const char* SENSOR_TCP_POSE = "tcp_pose";
inline const char* SENSOR_TCP_TWIST = "tcp_twist";
inline const char* SENSOR_TCP_WRENCH = "tcp_wrench";
inline const char* SENSOR_BASE_POSE = "base_pose";
inline const char* SENSOR_BASE_TWIST = "base_twist";

/**
 * @brief Publication of the sensor values on ROS 2 topics (see ros_publisher.hpp for the topics and messages).
 */
struct SensorPublishingConfig {
    std::string topic_namespace;  ///< Prefix of all the sensor topics
    double publish_period;        ///< Period in seconds of the publication

    // Built-in sensors to publish (resolved from the "sensors" list, or all available ones if the list is omitted)
    bool joint_state = false;
    bool tcp_pose = false;
    bool tcp_twist = false;
    bool tcp_wrench = false;
    bool base_pose = false;
    bool base_twist = false;

    bool all_mujoco_sensors = false;            ///< true if the "sensors" list is omitted
    std::vector<std::string> mujoco_sensors;    ///< MuJoCo sensors (MJCF) explicitly listed in "sensors"
};

/**
 * @brief Optional per-joint overrides of the joint dynamics read from the URDF.
 *        Each vector is either empty (use the URDF value) or has one entry per robot joint.
 */
struct JointDynamicsOverride {
    std::vector<double> armature;
    std::vector<double> damping;
    std::vector<double> frictionloss;
};

/**
 * @brief Selects which feedback channels of crospi are enabled (presence of name_expr_* keys in the configuration).
 */
struct FeedbackRequest {
    bool joint_vel = false;
    bool joint_torque = false;
    bool cartesian_pos = false;
    bool cartesian_quat = false;
    bool cartesian_twist = false;
    bool cartesian_wrench = false;
    bool base_pos = false;
    bool base_quat = false;
    bool base_twist = false;
};

/**
 * @brief All the parameters of the MuJoCo robot simulator. Every value is read from the crospi JSON configuration
 *        (see json_schemas/mujoco_robotsimulator_crospi.schema.json). There are no hidden default values: required
 *        parameters must be present and optional parameters only enable optional features.
 */
struct SimulatorConfig {
    // Crospi interface
    std::vector<std::string> robot_joints;
    std::vector<double> initial_joints;
    double periodicity;

    // Model
    std::string urdf_path;                          ///< Resolved (absolute) path of the URDF file
    std::optional<std::string> environment_mjcf_path;  ///< Resolved path of an optional MJCF scene in which the robot is placed
    std::array<double, 3> base_position;            ///< Position of the URDF root link in the world [m]
    std::array<double, 4> base_quaternion_xyzw;     ///< Orientation of the URDF root link in the world
    bool add_ground_plane;

    // Physics
    double simulation_timestep;
    std::string integrator;                         ///< "Euler", "RK4", "implicit" or "implicitfast"
    std::array<double, 3> gravity;
    bool contacts_enabled;
    JointDynamicsOverride joint_dynamics;

    // Control
    ControlMode control_mode;
    std::vector<double> joint_stiffness;            ///< Kp [Nm/rad or N/m], only for POSITION_SERVO
    std::vector<double> joint_damping_gains;        ///< Kd [Nms/rad or Ns/m], for POSITION_SERVO and VELOCITY_SERVO
    bool gravity_compensation;                      ///< only for dynamic modes
    bool clamp_torques_to_effort_limits;            ///< only for dynamic modes

    // Frames used for Cartesian/base feedback
    std::optional<std::string> base_link;
    std::optional<std::string> tcp_link;

    FeedbackRequest feedback;

    // ROS 2 interface
    std::string ros_node_name;                      ///< Node created by the simulator for publishing
    SensorPublishingConfig sensor_publishing;
    std::optional<VisualizationConfig> visualization;
    std::optional<ExternalWrenchConfig> external_wrench;

    /** Number of physics steps executed every update() call: periodicity / simulation_timestep */
    int substeps;
};

/**
 * @brief Reads and validates the configuration of one entry of robot/robotsimulators.
 * @throws std::runtime_error with a descriptive message if the configuration is invalid.
 */
SimulatorConfig parse_config(const Json::Value& config, std::shared_ptr<etasl::JsonChecker> jsonchecker);

/**
 * @brief Resolves a path that may contain $[package] / ${ENV} (crospi string interpolation), package://package/...
 *        or file:// prefixes. Relative paths are resolved with respect to base_dir (if not empty).
 */
std::string resolve_path(const std::string& path, const std::string& base_dir);

} // namespace mujoco_sim
} // namespace etasl
