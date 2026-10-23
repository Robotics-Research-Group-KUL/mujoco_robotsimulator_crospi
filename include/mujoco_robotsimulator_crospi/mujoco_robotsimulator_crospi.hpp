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

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "robot_interfacing_utils/robotsimulator.hpp"

#include "mujoco_robotsimulator_crospi/mujoco_simulator_config.hpp"
#include "mujoco_robotsimulator_crospi/ros_publisher.hpp"


namespace etasl {

/**
 * @brief Crospi robot simulator based on the MuJoCo physics engine.
 *
 * The robot is loaded from a URDF file and simulated in MuJoCo. Every update() call (executed every `periodicity`
 * seconds by the crospi driver thread) reads the joint velocity setpoint, advances the simulation by `periodicity`
 * seconds (in `periodicity / simulation_timestep` physics steps) and writes the feedback. See README.md and
 * json_schemas/mujoco_robotsimulator_crospi.schema.json for the configuration.
 */
class mujoco_robotsimulator_crospi : public RobotSimulator {
    public:
        typedef std::shared_ptr<mujoco_robotsimulator_crospi> SharedPtr;

    private:
        // FeedbackMsg* feedback_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // SetpointMsg* setpoint_ptr; Defined in super class RobotDriver at header file robotdriver.hpp
        // std::string name;; Defined in super class RobotDriver at header file robotdriver.hpp

        mujoco_sim::SimulatorConfig config_;

        mjModel* m_ = nullptr;
        mjData* d_ = nullptr;
        std::unique_ptr<mujoco_sim::RosPublisher> ros_publisher_;

        // Indices of the robot_joints in the MuJoCo model
        std::vector<int> jnt_id_;
        std::vector<int> qpos_adr_;
        std::vector<int> dof_adr_;

        // Ids used for Cartesian/base feedback (-1 if not used)
        int base_body_id_ = -1;
        int tcp_body_id_ = -1;
        std::vector<bool> end_effector_body_;  // bodies whose contacts contribute to the tcp wrench

        // Contact forces for visualization (optional, visualization/contact_topic)
        std::vector<double> contacts_;          // preallocated: max_contacts(m) * CONTACT_STRIDE
        std::size_t n_contacts_ = 0;
        std::vector<bool> robot_moving_body_;   // bodies moved by the robot joints (and the bodies after them)

        // External wrench (optional input, e.g. mouse perturbations of the viewer)
        mujoco_sim::ExternalWrench external_wrench_;   // latest received wrench (body frame, at the body origin)
        int external_wrench_body_ = -1;                // body to which a wrench is currently applied (-1: none)
        std::array<double, 3> external_force_world_{}; // applied wrench, world orientation, at the body origin
        std::array<double, 3> external_torque_world_{};

        // Quantities to compute every update (for the crospi feedback and/or the published sensors)
        bool need_tau_ = false;
        bool need_tcp_pose_ = false;
        bool need_tcp_twist_ = false;
        bool need_wrench_ = false;
        bool need_base_pose_ = false;
        bool need_base_twist_ = false;
        mujoco_sim::Measurements meas_;

        // Controller state
        std::vector<double> q_ref_;            // integrated setpoint (kinematic and position_servo modes)
        std::vector<double> tau_;              // torques applied by the joint controller in the last physics step
        std::vector<double> qvel_prev_;        // used to compute the joint torques in kinematic mode
        std::atomic<bool> resync_reference_{false};

        // Data exchanged with crospi
        robotdrivers::DynamicJointDataField setpoint_joint_vel_;
        robotdrivers::DynamicJointDataField joint_pos_;
        robotdrivers::DynamicJointDataField joint_vel_;
        robotdrivers::DynamicJointDataField joint_torque_;

        void map_joints();
        void reset_state();
        void step_kinematic();
        void step_dynamic();
        void compute_measurements();
        void compute_tcp_wrench();
        void apply_external_wrench();
        void compute_contacts();
        void write_feedback();

    public:
        mujoco_robotsimulator_crospi();

        virtual void construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        /**
         * Sets the robot at initial_joints and writes the initial feedback.
        */
        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;

        virtual ~mujoco_robotsimulator_crospi();
};

} // namespace etasl
