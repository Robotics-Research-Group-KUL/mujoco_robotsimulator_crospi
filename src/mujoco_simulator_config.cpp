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

#include "mujoco_robotsimulator_crospi/mujoco_simulator_config.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <fmt/format.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "crospi_utils/string_interpolate.hpp"

namespace etasl {
namespace mujoco_sim {

namespace {

const std::string PLUGIN_NAME = "mujoco_robotsimulator_crospi";

[[noreturn]] void fail(const std::string& msg) {
    throw std::runtime_error(fmt::format("[{}] Invalid configuration: {}", PLUGIN_NAME, msg));
}

const Json::Value& require(const Json::Value& config, const std::string& key) {
    if (!config.isObject() || !config.isMember(key)) {
        fail(fmt::format("the required parameter '{}' is missing.", key));
    }
    return config[key];
}

double read_number(const Json::Value& value, const std::string& key) {
    if (!value.isNumeric()) {
        fail(fmt::format("parameter '{}' must be a number.", key));
    }
    return value.asDouble();
}

bool read_bool(const Json::Value& config, const std::string& key) {
    const Json::Value& v = require(config, key);
    if (!v.isBool()) {
        fail(fmt::format("parameter '{}' must be a boolean.", key));
    }
    return v.asBool();
}

std::string read_string(const Json::Value& config, const std::string& key) {
    const Json::Value& v = require(config, key);
    if (!v.isString()) {
        fail(fmt::format("parameter '{}' must be a string.", key));
    }
    return v.asString();
}

std::vector<double> read_number_array(const Json::Value& config, const std::string& key) {
    const Json::Value& v = require(config, key);
    if (!v.isArray()) {
        fail(fmt::format("parameter '{}' must be an array of numbers.", key));
    }
    std::vector<double> result;
    for (const auto& item : v) {
        result.push_back(read_number(item, key));
    }
    return result;
}

template <std::size_t N>
std::array<double, N> read_fixed_array(const Json::Value& config, const std::string& key) {
    std::vector<double> v = read_number_array(config, key);
    if (v.size() != N) {
        fail(fmt::format("parameter '{}' must have exactly {} elements (got {}).", key, N, v.size()));
    }
    std::array<double, N> result;
    std::copy(v.begin(), v.end(), result.begin());
    return result;
}

std::vector<double> read_joint_array(const Json::Value& config, const std::string& key, std::size_t njoints) {
    std::vector<double> v = read_number_array(config, key);
    if (v.size() != njoints) {
        fail(fmt::format("parameter '{}' must have one value per joint in robot_joints ({} values expected, got {}).",
                         key, njoints, v.size()));
    }
    return v;
}

void forbid(const Json::Value& config, const std::string& key, const std::string& reason) {
    if (config.isMember(key)) {
        fail(fmt::format("parameter '{}' is not allowed {}.", key, reason));
    }
}

} // namespace


std::string resolve_path(const std::string& path, const std::string& base_dir)
{
    std::string p = etasl::string_interpolate(path);  // $[package] and ${ENV}

    const std::string package_prefix = "package://";
    const std::string file_prefix = "file://";
    if (p.rfind(package_prefix, 0) == 0) {
        std::string rest = p.substr(package_prefix.size());
        std::size_t slash = rest.find('/');
        std::string package = rest.substr(0, slash);
        std::string relative = (slash == std::string::npos) ? "" : rest.substr(slash + 1);
        std::string share;
        try {
            share = ament_index_cpp::get_package_share_directory(package);
        } catch (const std::exception& e) {
            throw std::runtime_error(fmt::format("[{}] Could not resolve '{}': package '{}' not found ({}).",
                                                 PLUGIN_NAME, path, package, e.what()));
        }
        p = (std::filesystem::path(share) / relative).string();
    } else if (p.rfind(file_prefix, 0) == 0) {
        p = p.substr(file_prefix.size());
    }

    std::filesystem::path fs_path(p);
    if (fs_path.is_relative() && !base_dir.empty()) {
        fs_path = std::filesystem::path(base_dir) / fs_path;
    }
    return fs_path.lexically_normal().string();
}


SimulatorConfig parse_config(const Json::Value& config, std::shared_ptr<etasl::JsonChecker> /*jsonchecker*/)
{
    SimulatorConfig c;

    // ---------------- crospi interface ----------------
    const Json::Value joints = require(config, "robot_joints");
    if (!joints.isArray() || joints.empty()) {
        fail("parameter 'robot_joints' must be a non-empty array of joint names.");
    }
    for (const auto& j : joints) {
        if (!j.isString()) {
            fail("parameter 'robot_joints' must only contain strings.");
        }
        c.robot_joints.push_back(j.asString());
    }
    const std::size_t n = c.robot_joints.size();
    c.initial_joints = read_joint_array(config, "initial_joints", n);
    c.periodicity = read_number(require(config, "periodicity"), "periodicity");
    if (c.periodicity <= 0.0) {
        fail("parameter 'periodicity' must be strictly positive.");
    }

    // ---------------- model ----------------
    c.urdf_path = resolve_path(read_string(config, "urdf_path"), "");
    if (!std::filesystem::is_regular_file(c.urdf_path)) {
        fail(fmt::format("the URDF file '{}' (urdf_path) does not exist.", c.urdf_path));
    }
    if (config.isMember("environment_mjcf_path")) {
        c.environment_mjcf_path = resolve_path(read_string(config, "environment_mjcf_path"), "");
        if (!std::filesystem::is_regular_file(*c.environment_mjcf_path)) {
            fail(fmt::format("the MJCF file '{}' (environment_mjcf_path) does not exist.", *c.environment_mjcf_path));
        }
    }
    c.base_position = read_fixed_array<3>(config, "base_position");
    c.base_quaternion_xyzw = read_fixed_array<4>(config, "base_quaternion_xyzw");
    double qnorm = std::sqrt(c.base_quaternion_xyzw[0] * c.base_quaternion_xyzw[0] +
                             c.base_quaternion_xyzw[1] * c.base_quaternion_xyzw[1] +
                             c.base_quaternion_xyzw[2] * c.base_quaternion_xyzw[2] +
                             c.base_quaternion_xyzw[3] * c.base_quaternion_xyzw[3]);
    if (std::abs(qnorm - 1.0) > 1e-3) {
        fail(fmt::format("parameter 'base_quaternion_xyzw' must be a unit quaternion (norm is {}).", qnorm));
    }
    c.add_ground_plane = read_bool(config, "add_ground_plane");

    // ---------------- physics ----------------
    c.simulation_timestep = read_number(require(config, "simulation_timestep"), "simulation_timestep");
    if (c.simulation_timestep <= 0.0) {
        fail("parameter 'simulation_timestep' must be strictly positive.");
    }
    double ratio = c.periodicity / c.simulation_timestep;
    c.substeps = static_cast<int>(std::lround(ratio));
    if (c.substeps < 1 || std::abs(ratio - c.substeps) > 1e-6 * ratio) {
        fail(fmt::format("'periodicity' ({}) must be an integer multiple of 'simulation_timestep' ({}).",
                         c.periodicity, c.simulation_timestep));
    }
    c.integrator = read_string(config, "integrator");
    if (c.integrator != "Euler" && c.integrator != "RK4" && c.integrator != "implicit" && c.integrator != "implicitfast") {
        fail(fmt::format("parameter 'integrator' must be one of Euler, RK4, implicit, implicitfast (got '{}').", c.integrator));
    }
    c.gravity = read_fixed_array<3>(config, "gravity");
    c.contacts_enabled = read_bool(config, "contacts_enabled");

    if (config.isMember("joint_armature")) {
        c.joint_dynamics.armature = read_joint_array(config, "joint_armature", n);
    }
    if (config.isMember("joint_damping")) {
        c.joint_dynamics.damping = read_joint_array(config, "joint_damping", n);
    }
    if (config.isMember("joint_frictionloss")) {
        c.joint_dynamics.frictionloss = read_joint_array(config, "joint_frictionloss", n);
    }

    // ---------------- control ----------------
    std::string mode = read_string(config, "control_mode");
    if (mode == "kinematic") {
        c.control_mode = ControlMode::KINEMATIC;
        const std::string reason = "when control_mode is 'kinematic' (it only applies to the dynamic control modes)";
        forbid(config, "joint_stiffness", reason);
        forbid(config, "joint_damping_gains", reason);
        forbid(config, "gravity_compensation", reason);
        forbid(config, "clamp_torques_to_effort_limits", reason);
        c.gravity_compensation = false;
        c.clamp_torques_to_effort_limits = false;
    } else if (mode == "position_servo" || mode == "velocity_servo") {
        if (mode == "position_servo") {
            c.control_mode = ControlMode::POSITION_SERVO;
            c.joint_stiffness = read_joint_array(config, "joint_stiffness", n);
        } else {
            c.control_mode = ControlMode::VELOCITY_SERVO;
            forbid(config, "joint_stiffness", "when control_mode is 'velocity_servo'");
        }
        c.joint_damping_gains = read_joint_array(config, "joint_damping_gains", n);
        c.gravity_compensation = read_bool(config, "gravity_compensation");
        c.clamp_torques_to_effort_limits = read_bool(config, "clamp_torques_to_effort_limits");
    } else {
        fail(fmt::format("parameter 'control_mode' must be one of kinematic, position_servo, velocity_servo (got '{}').", mode));
    }

    // ---------------- feedback ----------------
    if (config.isMember("base_link")) {
        c.base_link = read_string(config, "base_link");
    }
    if (config.isMember("tcp_link")) {
        c.tcp_link = read_string(config, "tcp_link");
    }
    c.feedback.joint_vel = config.isMember("name_expr_joint_vel");
    c.feedback.joint_torque = config.isMember("name_expr_joint_torque");
    c.feedback.cartesian_pos = config.isMember("name_expr_cartesian_pos");
    c.feedback.cartesian_quat = config.isMember("name_expr_cartesian_quat");
    c.feedback.cartesian_twist = config.isMember("name_expr_cartesian_twist");
    c.feedback.cartesian_wrench = config.isMember("name_expr_cartesian_wrench");
    c.feedback.base_pos = config.isMember("name_expr_base_pos");
    c.feedback.base_quat = config.isMember("name_expr_base_quat");
    c.feedback.base_twist = config.isMember("name_expr_base_twist");
    if (config.isMember("name_expr_joint_current")) {
        fail("'name_expr_joint_current' is not supported by the MuJoCo simulator (motor currents are not simulated).");
    }

    bool needs_base = c.feedback.cartesian_pos || c.feedback.cartesian_quat || c.feedback.cartesian_twist ||
                      c.feedback.base_pos || c.feedback.base_quat || c.feedback.base_twist;
    bool needs_tcp = c.feedback.cartesian_pos || c.feedback.cartesian_quat || c.feedback.cartesian_twist ||
                     c.feedback.cartesian_wrench;
    if (needs_base && !c.base_link) {
        fail("'base_link' is required when Cartesian (pos/quat/twist) or base feedback is requested.");
    }
    if (needs_tcp && !c.tcp_link) {
        fail("'tcp_link' is required when Cartesian feedback is requested.");
    }

    // ---------------- visualization ----------------
    if (config.isMember("visualization")) {
        const Json::Value& v = config["visualization"];
        VisualizationConfig vis;
        vis.node_name = read_string(v, "node_name");
        vis.model_topic = read_string(v, "model_topic");
        vis.state_topic = read_string(v, "state_topic");
        vis.publish_period = read_number(require(v, "publish_period"), "visualization/publish_period");
        if (vis.publish_period <= 0.0) {
            fail("parameter 'visualization/publish_period' must be strictly positive.");
        }
        c.visualization = vis;
    }

    return c;
}

} // namespace mujoco_sim
} // namespace etasl
