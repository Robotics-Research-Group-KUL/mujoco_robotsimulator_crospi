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
#include <chrono>
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

robotdrivers::Vector3Field to_vector3(const std::array<double, 3>& v) {
    robotdrivers::Vector3Field f{};
    f.x = static_cast<float>(v[0]);
    f.y = static_cast<float>(v[1]);
    f.z = static_cast<float>(v[2]);
    return f;
}

/** MuJoCo quaternion [w x y z] to crospi quaternion. */
robotdrivers::QuaternionField to_quaternion(const std::array<double, 4>& q) {
    robotdrivers::QuaternionField f{};
    f.qw = static_cast<float>(q[0]);
    f.qx = static_cast<float>(q[1]);
    f.qy = static_cast<float>(q[2]);
    f.qz = static_cast<float>(q[3]);
    return f;
}

robotdrivers::ScrewField to_screw(const std::array<double, 3>& linear, const std::array<double, 3>& angular) {
    robotdrivers::ScrewField f{};
    f.linear = to_vector3(linear);
    f.angular = to_vector3(angular);
    return f;
}

void copy3(std::array<double, 3>& dst, const mjtNum* src) {
    std::copy(src, src + 3, dst.begin());
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
    const auto& fb = config_.feedback;
    const auto& pub = config_.sensor_publishing;
    need_tau_ = fb.joint_torque || pub.joint_state;
    need_tcp_pose_ = fb.cartesian_pos || fb.cartesian_quat || pub.tcp_pose;
    need_tcp_twist_ = fb.cartesian_twist || pub.tcp_twist;
    need_wrench_ = fb.cartesian_wrench || pub.tcp_wrench;
    need_base_pose_ = fb.base_pos || fb.base_quat || pub.base_pose;
    need_base_twist_ = fb.base_twist || pub.base_twist;

    if (need_wrench_) {
        // End effector: the rigid body group of tcp_link (bodies welded together, e.g. wrist_3_link, flange, tool0)
        // and everything mounted after it (e.g. a gripper with its own joints).
        const int root = m_->body_weldid[tcp_body_id_];
        end_effector_body_.assign(m_->nbody, false);
        for (int b = 1; b < m_->nbody; ++b) {
            for (int a = b; a > 0; a = m_->body_parentid[a]) {
                if (a == root) {
                    end_effector_body_[b] = true;
                    break;
                }
            }
        }
    }

    const std::size_t n = config_.robot_joints.size();
    setpoint_joint_vel_.data.assign(n, 0.0f);
    joint_pos_.data.assign(n, 0.0f);
    joint_vel_.data.assign(n, 0.0f);
    joint_torque_.data.assign(n, 0.0f);
    q_ref_.assign(n, 0.0);
    tau_.assign(n, 0.0);
    meas_.q.assign(n, 0.0);
    meas_.qd.assign(n, 0.0);
    meas_.tau.assign(n, 0.0);
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

    ros_publisher_ = std::make_unique<mujoco_sim::RosPublisher>(config_, m_);

    if (ros_publisher_->publishes_contacts()) {
        contacts_.assign(mujoco_sim::max_contacts(m_) * mujoco_sim::CONTACT_STRIDE, 0.0);
        // Moving bodies of the robot: the bodies of the robot joints and all the bodies after them.
        robot_moving_body_.assign(m_->nbody, false);
        for (int j : jnt_id_) {
            const int joint_body = m_->jnt_bodyid[j];
            for (int b = 1; b < m_->nbody; ++b) {
                for (int a = b; a > 0; a = m_->body_parentid[a]) {
                    if (a == joint_body) {
                        robot_moving_body_[b] = true;
                        break;
                    }
                }
            }
        }
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
    external_wrench_ = mujoco_sim::ExternalWrench{};
    external_wrench_body_ = -1;
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

    compute_measurements();
    write_feedback();
    compute_contacts();
    ros_publisher_->store(d_, meas_, contacts_, n_contacts_);
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

    if (config_.external_wrench) {
        apply_external_wrench();
    }

    if (config_.control_mode == mujoco_sim::ControlMode::KINEMATIC) {
        step_kinematic();
    } else {
        step_dynamic();
    }

    compute_measurements();
    write_feedback();
    compute_contacts();
    ros_publisher_->store(d_, meas_, contacts_, n_contacts_);
}

void mujoco_robotsimulator_crospi::apply_external_wrench()
{
    // Latest wrench received on external_wrench/topic (body frame, reference point at the body origin).
    mujoco_sim::ExternalWrench received;
    if (ros_publisher_->fetch_external_wrench(received)) {
        external_wrench_ = received;
    }

    // Remove the wrench applied during the previous update.
    if (external_wrench_body_ >= 0) {
        mju_zero(d_->xfrc_applied + 6 * external_wrench_body_, 6);
        external_wrench_body_ = -1;
    }

    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - external_wrench_.received).count();
    if (external_wrench_.body <= 0 || age > config_.external_wrench->timeout) {
        return;  // no wrench, or too old (e.g. the publisher stopped)
    }

    // Limit the norms of the force and torque.
    mjtNum f_body[3], t_body[3];
    std::copy(external_wrench_.force.begin(), external_wrench_.force.end(), f_body);
    std::copy(external_wrench_.torque.begin(), external_wrench_.torque.end(), t_body);
    const mjtNum f_norm = mju_norm3(f_body);
    const mjtNum t_norm = mju_norm3(t_body);
    if (f_norm > config_.external_wrench->max_force) {
        mju_scl3(f_body, f_body, config_.external_wrench->max_force / f_norm);
    }
    if (t_norm > config_.external_wrench->max_torque) {
        mju_scl3(t_body, t_body, config_.external_wrench->max_torque / t_norm);
    }

    // Body frame -> world orientation, then MuJoCo convention of xfrc_applied: torque about the center of mass.
    const int b = external_wrench_.body;
    mjtNum f_world[3], t_world[3], com_to_origin[3], moment[3];
    mju_mulMatVec3(f_world, d_->xmat + 9 * b, f_body);
    mju_mulMatVec3(t_world, d_->xmat + 9 * b, t_body);
    mju_sub3(com_to_origin, d_->xpos + 3 * b, d_->xipos + 3 * b);
    mju_cross(moment, com_to_origin, f_world);
    mjtNum* xfrc = d_->xfrc_applied + 6 * b;
    mju_copy3(xfrc, f_world);
    mju_add3(xfrc + 3, t_world, moment);

    external_wrench_body_ = b;
    copy3(external_force_world_, f_world);
    copy3(external_torque_world_, t_world);
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

    if (need_tau_) {
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

    // After mj_step, the positions are advanced but the derived quantities (body poses, sensors, gravity
    // compensation) still correspond to the previous state: recompute them for the feedback and the sensors.
    mj_forward(m_, d_);
}

void mujoco_robotsimulator_crospi::compute_measurements()
{
    const std::size_t n = qpos_adr_.size();
    const bool dynamic = (config_.control_mode != mujoco_sim::ControlMode::KINEMATIC);
    for (std::size_t i = 0; i < n; ++i) {
        meas_.q[i] = d_->qpos[qpos_adr_[i]];
        meas_.qd[i] = d_->qvel[dof_adr_[i]];
        // In the dynamic modes, the motor torque is the controller torque plus the gravity compensation.
        meas_.tau[i] = tau_[i] + (dynamic ? d_->qfrc_gravcomp[dof_adr_[i]] : 0.0);
    }

    if (need_tcp_pose_ || need_tcp_twist_) {
        const mjtNum* p_b = d_->xpos + 3 * base_body_id_;
        const mjtNum* R_b = d_->xmat + 9 * base_body_id_;
        const mjtNum* p_t = d_->xpos + 3 * tcp_body_id_;
        mjtNum r[3];
        mju_sub3(r, p_t, p_b);

        if (need_tcp_pose_) {
            mjtNum p_rel[3], q_b_inv[4], q_rel[4];
            mju_mulMatTVec3(p_rel, R_b, r);
            copy3(meas_.tcp_pos, p_rel);
            mju_negQuat(q_b_inv, d_->xquat + 4 * base_body_id_);
            mju_mulQuat(q_rel, q_b_inv, d_->xquat + 4 * tcp_body_id_);
            std::copy(q_rel, q_rel + 4, meas_.tcp_quat.begin());
        }
        if (need_tcp_twist_) {
            // Velocities [angular; linear] at the body origins, in world orientation
            mjtNum v_t[6], v_b[6];
            mj_objectVelocity(m_, d_, mjOBJ_BODY, tcp_body_id_, v_t, 0);
            mj_objectVelocity(m_, d_, mjOBJ_BODY, base_body_id_, v_b, 0);
            // Twist of the tcp relative to the base, at the tcp origin, expressed in the base frame
            mjtNum w_cross_r[3], w_rel[3], v_rel[3];
            mju_cross(w_cross_r, v_b, r);
            mju_sub3(w_rel, v_t, v_b);
            for (int k = 0; k < 3; ++k) {
                v_rel[k] = v_t[3 + k] - v_b[3 + k] - w_cross_r[k];
            }
            mjtNum w_base[3], v_base[3];
            mju_mulMatTVec3(w_base, R_b, w_rel);
            mju_mulMatTVec3(v_base, R_b, v_rel);
            copy3(meas_.tcp_ang_vel, w_base);
            copy3(meas_.tcp_lin_vel, v_base);
        }
    }

    if (need_wrench_) {
        compute_tcp_wrench();
    }

    if (need_base_pose_) {
        copy3(meas_.base_pos, d_->xpos + 3 * base_body_id_);
        std::copy(d_->xquat + 4 * base_body_id_, d_->xquat + 4 * base_body_id_ + 4, meas_.base_quat.begin());
    }
    if (need_base_twist_) {
        mjtNum v_b[6];
        mj_objectVelocity(m_, d_, mjOBJ_BODY, base_body_id_, v_b, 0);
        copy3(meas_.base_ang_vel, v_b);
        copy3(meas_.base_lin_vel, v_b + 3);
    }
}

void mujoco_robotsimulator_crospi::compute_tcp_wrench()
{
    // Sum of the contact wrenches exerted by the environment on the end effector, with reference point at the
    // origin of tcp_link (world orientation).
    const mjtNum* p_t = d_->xpos + 3 * tcp_body_id_;
    mjtNum force_env[3] = {0, 0, 0};
    mjtNum torque_env[3] = {0, 0, 0};
    for (int i = 0; i < d_->ncon; ++i) {
        const mjContact& c = d_->contact[i];
        if (c.exclude != 0 || c.efc_address < 0) {
            continue;  // inactive contact
        }
        const bool on_ee1 = end_effector_body_[m_->geom_bodyid[c.geom[0]]];
        const bool on_ee2 = end_effector_body_[m_->geom_bodyid[c.geom[1]]];
        if (on_ee1 == on_ee2) {
            continue;  // not involving the end effector, or internal to it
        }
        // Contact frame -> world: force exerted by geom[0] on geom[1]
        mjtNum f_contact[6], f_world[3], t_world[3];
        mj_contactForce(m_, d_, i, f_contact);
        mju_mulMatTVec3(f_world, c.frame, f_contact);
        mju_mulMatTVec3(t_world, c.frame, f_contact + 3);
        const mjtNum sign = on_ee2 ? 1.0 : -1.0;  // force acting on the end effector
        mjtNum r[3], r_cross_f[3];
        mju_sub3(r, c.pos, p_t);
        mju_cross(r_cross_f, r, f_world);
        for (int k = 0; k < 3; ++k) {
            force_env[k] += sign * f_world[k];
            torque_env[k] += sign * (t_world[k] + r_cross_f[k]);
        }
    }
    // External wrench applied to the end effector (e.g. a user pushing the tool): measured as well, as by a real
    // force/torque sensor between the robot and the tool.
    if (external_wrench_body_ >= 0 && end_effector_body_[external_wrench_body_]) {
        mjtNum r[3], r_cross_f[3];
        mju_sub3(r, d_->xpos + 3 * external_wrench_body_, p_t);
        mju_cross(r_cross_f, r, external_force_world_.data());
        for (int k = 0; k < 3; ++k) {
            force_env[k] += external_force_world_[k];
            torque_env[k] += external_torque_world_[k] + r_cross_f[k];
        }
    }
    // Reported: wrench exerted by the end effector on the environment, expressed in tcp_link.
    const mjtNum* R_t = d_->xmat + 9 * tcp_body_id_;
    mjtNum f_tcp[3], t_tcp[3];
    mju_mulMatTVec3(f_tcp, R_t, force_env);
    mju_mulMatTVec3(t_tcp, R_t, torque_env);
    for (int k = 0; k < 3; ++k) {
        meas_.tcp_force[k] = -f_tcp[k];
        meas_.tcp_torque[k] = -t_tcp[k];
    }
}

void mujoco_robotsimulator_crospi::compute_contacts()
{
    n_contacts_ = 0;
    if (contacts_.empty()) {
        return;  // contact forces are not published
    }
    const std::size_t capacity = contacts_.size() / mujoco_sim::CONTACT_STRIDE;
    for (int i = 0; i < d_->ncon && n_contacts_ < capacity; ++i) {
        const mjContact& c = d_->contact[i];
        if (c.exclude != 0 || c.efc_address < 0) {
            continue;  // inactive contact
        }
        const int b0 = m_->geom_bodyid[c.geom[0]];
        const int b1 = m_->geom_bodyid[c.geom[1]];
        // Body on which the displayed force acts: the robot if involved, otherwise the moving body, otherwise body 1.
        bool on_b0;
        if (robot_moving_body_[b0] != robot_moving_body_[b1]) {
            on_b0 = robot_moving_body_[b0];
        } else if ((m_->body_weldid[b0] == 0) != (m_->body_weldid[b1] == 0)) {
            on_b0 = (m_->body_weldid[b1] == 0);
        } else {
            on_b0 = false;
        }
        // Contact frame -> world: force exerted by geom[0] on geom[1]
        mjtNum f_contact[6], f_world[3];
        mj_contactForce(m_, d_, i, f_contact);
        mju_mulMatTVec3(f_world, c.frame, f_contact);
        if (on_b0) {
            mju_scl3(f_world, f_world, -1.0);
        }
        double* entry = contacts_.data() + n_contacts_ * mujoco_sim::CONTACT_STRIDE;
        entry[0] = on_b0 ? b0 : b1;
        entry[1] = on_b0 ? b1 : b0;
        std::copy(c.pos, c.pos + 3, entry + 2);
        std::copy(f_world, f_world + 3, entry + 5);
        ++n_contacts_;
    }
}

void mujoco_robotsimulator_crospi::write_feedback()
{
    const std::size_t n = qpos_adr_.size();
    for (std::size_t i = 0; i < n; ++i) {
        joint_pos_.data[i] = static_cast<float>(meas_.q[i]);
    }
    writeFeedbackJointPosition(joint_pos_);

    const auto& fb = config_.feedback;
    if (fb.joint_vel) {
        for (std::size_t i = 0; i < n; ++i) {
            joint_vel_.data[i] = static_cast<float>(meas_.qd[i]);
        }
        writeFeedbackJointVelocity(joint_vel_);
    }
    if (fb.joint_torque) {
        for (std::size_t i = 0; i < n; ++i) {
            joint_torque_.data[i] = static_cast<float>(meas_.tau[i]);
        }
        writeFeedbackJointTorque(joint_torque_);
    }
    if (fb.cartesian_pos) {
        writeFeedbackCartesianPosition(to_vector3(meas_.tcp_pos));
    }
    if (fb.cartesian_quat) {
        writeFeedbackCartesianQuaternion(to_quaternion(meas_.tcp_quat));
    }
    if (fb.cartesian_twist) {
        writeFeedbackCartesianTwist(to_screw(meas_.tcp_lin_vel, meas_.tcp_ang_vel));
    }
    if (fb.cartesian_wrench) {
        writeFeedbackCartesianWrench(to_screw(meas_.tcp_force, meas_.tcp_torque));
    }
    if (fb.base_pos) {
        writeFeedbackBasePosition(to_vector3(meas_.base_pos));
    }
    if (fb.base_quat) {
        writeFeedbackBaseQuaternion(to_quaternion(meas_.base_quat));
    }
    if (fb.base_twist) {
        writeFeedbackBaseTwist(to_screw(meas_.base_lin_vel, meas_.base_ang_vel));
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
    ros_publisher_.reset();
}


mujoco_robotsimulator_crospi::~mujoco_robotsimulator_crospi() {
    ros_publisher_.reset();
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
