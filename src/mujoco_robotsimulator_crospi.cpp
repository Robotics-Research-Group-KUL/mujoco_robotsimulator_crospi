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

#include "mujoco_robotsimulator_crospi/mujoco_robotsimulator_crospi.hpp"
#include "mujoco_robotsimulator_crospi/mujoco_scene_builder.hpp"

#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>
#include <utility>

#include <fmt/format.h>

namespace etasl {

namespace {

const char* control_mode_name(mujoco_sim::ControlMode mode) {
    switch (mode) {
        case mujoco_sim::ControlMode::KINEMATIC: return "kinematic";
        case mujoco_sim::ControlMode::POSITION_SERVO: return "position_servo";
        case mujoco_sim::ControlMode::VELOCITY_SERVO: return "velocity_servo";
    }
    return "unknown";
}

robotdrivers::Vector3Field to_vector3(const mjtNum v[3]) {
    robotdrivers::Vector3Field f{};
    f.x = static_cast<float>(v[0]);
    f.y = static_cast<float>(v[1]);
    f.z = static_cast<float>(v[2]);
    return f;
}

/** MuJoCo quaternion [w x y z] to crospi quaternion. */
robotdrivers::QuaternionField to_quaternion(const mjtNum q[4]) {
    robotdrivers::QuaternionField f{};
    f.qw = static_cast<float>(q[0]);
    f.qx = static_cast<float>(q[1]);
    f.qy = static_cast<float>(q[2]);
    f.qz = static_cast<float>(q[3]);
    return f;
}

/** MuJoCo 6D velocity [angular; linear] to crospi screw. */
robotdrivers::ScrewField to_screw(const mjtNum angular[3], const mjtNum linear[3]) {
    robotdrivers::ScrewField f{};
    f.linear = to_vector3(linear);
    f.angular = to_vector3(angular);
    return f;
}

} // namespace


mujoco_robotsimulator_crospi::mujoco_robotsimulator_crospi()
{
}

void mujoco_robotsimulator_crospi::construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    name = robot_name; //defined in RobotDriver super class.

    config_ = mujoco_sim::parse_config(config, jsonchecker);

    m_ = mujoco_sim::build_model(config_);
    d_ = mj_makeData(m_);
    if (!d_) {
        throw std::runtime_error("[mujoco_robotsimulator_crospi] Could not allocate mjData.");
    }

    map_joints();

    if (config_.base_link) {
        base_body_id_ = mj_name2id(m_, mjOBJ_BODY, config_.base_link->c_str());
    }
    if (config_.tcp_link) {
        tcp_body_id_ = mj_name2id(m_, mjOBJ_BODY, config_.tcp_link->c_str());
    }
    if (config_.feedback.cartesian_wrench) {
        ft_force_adr_ = m_->sensor_adr[mj_name2id(m_, mjOBJ_SENSOR, mujoco_sim::FT_FORCE_SENSOR_NAME)];
        ft_torque_adr_ = m_->sensor_adr[mj_name2id(m_, mjOBJ_SENSOR, mujoco_sim::FT_TORQUE_SENSOR_NAME)];
    }

    const std::size_t n = config_.robot_joints.size();
    setpoint_joint_vel_.data.assign(n, 0.0f);
    joint_pos_.data.assign(n, 0.0f);
    joint_vel_.data.assign(n, 0.0f);
    joint_torque_.data.assign(n, 0.0f);
    q_ref_.assign(n, 0.0);
    tau_.assign(n, 0.0);
    qvel_prev_.assign(n, 0.0);

    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;
    available_fb.joint_vel = config_.feedback.joint_vel;
    available_fb.joint_torque = config_.feedback.joint_torque;
    available_fb.cartesian_pos = config_.feedback.cartesian_pos;
    available_fb.cartesian_quat = config_.feedback.cartesian_quat;
    available_fb.cartesian_twist = config_.feedback.cartesian_twist;
    available_fb.cartesian_wrench = config_.feedback.cartesian_wrench;
    available_fb.base_pos = config_.feedback.base_pos;
    available_fb.base_quat = config_.feedback.base_quat;
    available_fb.base_twist = config_.feedback.base_twist;

    constructPorts(n, available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.

    if (config_.visualization) {
        visualization_ = std::make_unique<mujoco_sim::VisualizationPublisher>(*config_.visualization, m_);
    }

    std::cout << fmt::format("Constructed MuJoCo robot simulator '{}' (MuJoCo {}): urdf={}, control_mode={}, "
                             "timestep={} s, {} physics steps per update, nq={}, nv={}, nbody={}, ngeom={}",
                             name, mj_versionString(), config_.urdf_path, control_mode_name(config_.control_mode),
                             config_.simulation_timestep, config_.substeps, m_->nq, m_->nv, m_->nbody, m_->ngeom)
              << std::endl;
}

void mujoco_robotsimulator_crospi::map_joints()
{
    const std::size_t n = config_.robot_joints.size();
    jnt_id_.resize(n);
    qpos_adr_.resize(n);
    dof_adr_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::string& jname = config_.robot_joints[i];
        int id = mj_name2id(m_, mjOBJ_JOINT, jname.c_str());
        if (id < 0) {
            throw std::runtime_error(fmt::format("[mujoco_robotsimulator_crospi] Joint '{}' not found in the MuJoCo model.", jname));
        }
        if (m_->jnt_type[id] != mjJNT_HINGE && m_->jnt_type[id] != mjJNT_SLIDE) {
            throw std::runtime_error(fmt::format("[mujoco_robotsimulator_crospi] Joint '{}' must be revolute/continuous or prismatic.", jname));
        }
        jnt_id_[i] = id;
        qpos_adr_[i] = m_->jnt_qposadr[id];
        dof_adr_[i] = m_->jnt_dofadr[id];
    }
}

void mujoco_robotsimulator_crospi::reset_state()
{
    mj_resetData(m_, d_);
    for (std::size_t i = 0; i < qpos_adr_.size(); ++i) {
        d_->qpos[qpos_adr_[i]] = config_.initial_joints[i];
        q_ref_[i] = config_.initial_joints[i];
        tau_[i] = 0.0;
        qvel_prev_[i] = 0.0;
    }
    std::fill(setpoint_joint_vel_.data.begin(), setpoint_joint_vel_.data.end(), 0.0f);
    mj_forward(m_, d_);
}

bool mujoco_robotsimulator_crospi::initialize()
{
    reset_state();

    // Report the contacts at the initial configuration: unexpected ones reveal an initial configuration in collision.
    std::set<std::pair<int, int>> body_pairs;
    for (int i = 0; i < d_->ncon; ++i) {
        const mjContact& c = d_->contact[i];
        body_pairs.insert({m_->geom_bodyid[c.geom[0]], m_->geom_bodyid[c.geom[1]]});
    }
    for (const auto& [b1, b2] : body_pairs) {
        std::cout << fmt::format("[mujoco_robotsimulator_crospi] Contact at the initial configuration between "
                                 "body '{}' and body '{}'", mj_id2name(m_, mjOBJ_BODY, b1), mj_id2name(m_, mjOBJ_BODY, b2))
                  << std::endl;
    }

    write_feedback();
    if (visualization_) {
        visualization_->store_state(d_);
    }
    return true;
}


void mujoco_robotsimulator_crospi::update(volatile std::atomic<bool>& /*stopFlag*/)
{
    readSetpointJointVelocity(setpoint_joint_vel_); // keeps the previous setpoint if no new one is available
    if (setpoint_joint_vel_.data.size() != q_ref_.size()) {
        setpoint_joint_vel_.data.assign(q_ref_.size(), 0.0f);
    }

    if (resync_reference_.exchange(false)) {
        for (std::size_t i = 0; i < q_ref_.size(); ++i) {
            q_ref_[i] = d_->qpos[qpos_adr_[i]];
        }
    }

    if (config_.control_mode == mujoco_sim::ControlMode::KINEMATIC) {
        step_kinematic();
    } else {
        step_dynamic();
    }

    write_feedback();

    if (visualization_) {
        visualization_->store_state(d_);
    }
}

void mujoco_robotsimulator_crospi::step_kinematic()
{
    const double dt = m_->opt.timestep;
    const std::size_t n = q_ref_.size();

    // The robot follows the integrated setpoints exactly; the rest of the scene (e.g. free objects) is simulated.
    for (int s = 0; s < config_.substeps; ++s) {
        for (std::size_t i = 0; i < n; ++i) {
            q_ref_[i] += setpoint_joint_vel_.data[i] * dt;
            d_->qpos[qpos_adr_[i]] = q_ref_[i];
            d_->qvel[dof_adr_[i]] = setpoint_joint_vel_.data[i];
        }
        mj_step(m_, d_);
    }
    for (std::size_t i = 0; i < n; ++i) {
        d_->qpos[qpos_adr_[i]] = q_ref_[i];
        d_->qvel[dof_adr_[i]] = setpoint_joint_vel_.data[i];
    }
    mj_forward(m_, d_);

    if (config_.feedback.joint_torque) {
        // Joint torques required to execute the commanded motion (inverse dynamics), with the acceleration
        // estimated from the change of the velocity setpoint over one update period.
        for (std::size_t i = 0; i < n; ++i) {
            d_->qacc[dof_adr_[i]] = (setpoint_joint_vel_.data[i] - qvel_prev_[i]) / config_.periodicity;
        }
        mj_inverse(m_, d_);
        for (std::size_t i = 0; i < n; ++i) {
            tau_[i] = d_->qfrc_inverse[dof_adr_[i]];
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        qvel_prev_[i] = setpoint_joint_vel_.data[i];
    }
}

void mujoco_robotsimulator_crospi::step_dynamic()
{
    const double dt = m_->opt.timestep;
    const std::size_t n = q_ref_.size();
    const bool position_servo = (config_.control_mode == mujoco_sim::ControlMode::POSITION_SERVO);

    for (int s = 0; s < config_.substeps; ++s) {
        for (std::size_t i = 0; i < n; ++i) {
            const int dof = dof_adr_[i];
            const double qd_sp = setpoint_joint_vel_.data[i];
            const double qd = d_->qvel[dof];
            double tau = config_.joint_damping_gains[i] * (qd_sp - qd);
            if (position_servo) {
                q_ref_[i] += qd_sp * dt;
                tau += config_.joint_stiffness[i] * (q_ref_[i] - d_->qpos[qpos_adr_[i]]);
            }
            if (config_.clamp_torques_to_effort_limits && m_->jnt_actfrclimited[jnt_id_[i]]) {
                // The total motor torque (controller + gravity compensation) is limited to the URDF effort limit.
                const double gravcomp = d_->qfrc_gravcomp[dof];
                tau = std::clamp(tau,
                                 m_->jnt_actfrcrange[2 * jnt_id_[i]] - gravcomp,
                                 m_->jnt_actfrcrange[2 * jnt_id_[i] + 1] - gravcomp);
            }
            d_->qfrc_applied[dof] = tau;
            tau_[i] = tau;
        }
        mj_step(m_, d_);
    }

    // After mj_step, the positions are advanced but the derived quantities (body poses, sensors) still correspond
    // to the previous state. Recompute them if they are reported.
    const auto& fb = config_.feedback;
    if (fb.cartesian_pos || fb.cartesian_quat || fb.cartesian_twist || fb.cartesian_wrench ||
        fb.base_pos || fb.base_quat || fb.base_twist || fb.joint_torque) {
        mj_forward(m_, d_);
    }
}

void mujoco_robotsimulator_crospi::write_feedback()
{
    const std::size_t n = qpos_adr_.size();
    for (std::size_t i = 0; i < n; ++i) {
        joint_pos_.data[i] = static_cast<float>(d_->qpos[qpos_adr_[i]]);
    }
    writeFeedbackJointPosition(joint_pos_);

    const auto& fb = config_.feedback;

    if (fb.joint_vel) {
        for (std::size_t i = 0; i < n; ++i) {
            joint_vel_.data[i] = static_cast<float>(d_->qvel[dof_adr_[i]]);
        }
        writeFeedbackJointVelocity(joint_vel_);
    }

    if (fb.joint_torque) {
        const bool dynamic = (config_.control_mode != mujoco_sim::ControlMode::KINEMATIC);
        for (std::size_t i = 0; i < n; ++i) {
            // In the dynamic modes, the motor torque is the controller torque plus the gravity compensation.
            double tau = tau_[i] + (dynamic ? d_->qfrc_gravcomp[dof_adr_[i]] : 0.0);
            joint_torque_.data[i] = static_cast<float>(tau);
        }
        writeFeedbackJointTorque(joint_torque_);
    }

    if (fb.cartesian_pos || fb.cartesian_quat || fb.cartesian_twist) {
        const mjtNum* p_b = d_->xpos + 3 * base_body_id_;
        const mjtNum* R_b = d_->xmat + 9 * base_body_id_;
        const mjtNum* p_t = d_->xpos + 3 * tcp_body_id_;

        if (fb.cartesian_pos) {
            mjtNum diff[3], p_rel[3];
            mju_sub3(diff, p_t, p_b);
            mju_mulMatTVec3(p_rel, R_b, diff);
            writeFeedbackCartesianPosition(to_vector3(p_rel));
        }
        if (fb.cartesian_quat) {
            mjtNum q_b_inv[4], q_rel[4];
            mju_negQuat(q_b_inv, d_->xquat + 4 * base_body_id_);
            mju_mulQuat(q_rel, q_b_inv, d_->xquat + 4 * tcp_body_id_);
            writeFeedbackCartesianQuaternion(to_quaternion(q_rel));
        }
        if (fb.cartesian_twist) {
            // Velocities [angular; linear] at the body origins, in world orientation
            mjtNum v_t[6], v_b[6];
            mj_objectVelocity(m_, d_, mjOBJ_BODY, tcp_body_id_, v_t, 0);
            mj_objectVelocity(m_, d_, mjOBJ_BODY, base_body_id_, v_b, 0);
            // Twist of the tcp relative to the base, at the tcp origin, expressed in the base frame
            mjtNum r[3], w_cross_r[3], w_rel[3], v_rel[3];
            mju_sub3(r, p_t, p_b);
            mju_cross(w_cross_r, v_b, r);
            mju_sub3(w_rel, v_t, v_b);
            for (int k = 0; k < 3; ++k) {
                v_rel[k] = v_t[3 + k] - v_b[3 + k] - w_cross_r[k];
            }
            mjtNum w_base[3], v_base[3];
            mju_mulMatTVec3(w_base, R_b, w_rel);
            mju_mulMatTVec3(v_base, R_b, v_rel);
            writeFeedbackCartesianTwist(to_screw(w_base, v_base));
        }
    }

    if (fb.cartesian_wrench) {
        // MuJoCo force/torque sensors at tcp_link, expressed in the tcp_link frame.
        writeFeedbackCartesianWrench(to_screw(d_->sensordata + ft_torque_adr_, d_->sensordata + ft_force_adr_));
    }

    if (fb.base_pos) {
        writeFeedbackBasePosition(to_vector3(d_->xpos + 3 * base_body_id_));
    }
    if (fb.base_quat) {
        writeFeedbackBaseQuaternion(to_quaternion(d_->xquat + 4 * base_body_id_));
    }
    if (fb.base_twist) {
        mjtNum v_b[6];
        mj_objectVelocity(m_, d_, mjOBJ_BODY, base_body_id_, v_b, 0);
        writeFeedbackBaseTwist(to_screw(v_b, v_b + 3));
    }
}

void mujoco_robotsimulator_crospi::on_configure() {
}

void mujoco_robotsimulator_crospi::on_activate()
{
    // Avoid wind-up of the position reference: the new task starts from the current (simulated) robot position.
    resync_reference_ = true;
}

void mujoco_robotsimulator_crospi::on_deactivate() {
}

void mujoco_robotsimulator_crospi::on_cleanup() {
}


void mujoco_robotsimulator_crospi::finalize() {
    std::cout << "finalize() called for MuJoCo robot simulator '" << name << "'" << std::endl;
    visualization_.reset();
}


mujoco_robotsimulator_crospi::~mujoco_robotsimulator_crospi() {
    visualization_.reset();
    if (d_) {
        mj_deleteData(d_);
    }
    if (m_) {
        mj_deleteModel(m_);
    }
}


} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::mujoco_robotsimulator_crospi, etasl::RobotDriver)
