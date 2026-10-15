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
#include <mutex>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "mujoco_robotsimulator_crospi/mujoco_simulator_config.hpp"

namespace etasl {
namespace mujoco_sim {

/**
 * @brief Publishes the simulation so that it can be visualized by an external viewer (scripts/mujoco_viewer.py):
 *  - model_topic (std_msgs/UInt8MultiArray, transient local): the compiled mjModel in MJB binary format, published once.
 *  - state_topic (std_msgs/Float64MultiArray): [simulation time, qpos[0..nq-1]] published every publish_period seconds.
 *
 * It runs its own node and executor in a separate thread. The real-time update thread only copies the state into
 * a buffer with a non-blocking try_lock, so it is never blocked by ROS.
 */
class VisualizationPublisher {
    public:
        VisualizationPublisher(const VisualizationConfig& config, const mjModel* m);
        ~VisualizationPublisher();

        /** Called from the update thread. Never blocks: the sample is skipped if the publisher holds the buffer. */
        void store_state(const mjData* d) noexcept;

    private:
        void publish_state();

        rclcpp::Node::SharedPtr node_;
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr model_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::thread spin_thread_;

        std::mutex buffer_mutex_;
        std::vector<double> buffer_;   // [time, qpos...]
        bool buffer_updated_ = false;
        std_msgs::msg::Float64MultiArray state_msg_;
};

} // namespace mujoco_sim
} // namespace etasl
