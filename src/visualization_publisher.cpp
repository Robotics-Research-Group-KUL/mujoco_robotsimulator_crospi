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

#include "mujoco_robotsimulator_crospi/visualization_publisher.hpp"

#include <chrono>
#include <stdexcept>

#include <fmt/format.h>

namespace etasl {
namespace mujoco_sim {

VisualizationPublisher::VisualizationPublisher(const VisualizationConfig& config, const mjModel* m)
{
    if (!rclcpp::ok()) {
        throw std::runtime_error("[mujoco_robotsimulator_crospi] 'visualization' requires an initialized ROS 2 context (rclcpp::init).");
    }
    node_ = std::make_shared<rclcpp::Node>(config.node_name);

    // Model: published once, kept for late-joining subscribers.
    auto model_qos = rclcpp::QoS(1).reliable().transient_local();
    model_pub_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(config.model_topic, model_qos);
    std_msgs::msg::UInt8MultiArray model_msg;
    mjtSize size = mj_sizeModel(m);
    model_msg.data.resize(static_cast<std::size_t>(size));
    mj_saveModel(m, nullptr, model_msg.data.data(), size);
    model_pub_->publish(model_msg);

    // State
    state_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(config.state_topic, rclcpp::QoS(1));
    buffer_.assign(1 + m->nq, 0.0);
    state_msg_.data.assign(1 + m->nq, 0.0);
    std_msgs::msg::MultiArrayDimension dim;
    dim.label = "time_and_qpos";
    dim.size = 1 + m->nq;
    dim.stride = 1 + m->nq;
    state_msg_.layout.dim.push_back(dim);

    timer_ = node_->create_wall_timer(std::chrono::duration<double>(config.publish_period),
                                      [this]() { publish_state(); });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    RCLCPP_INFO(node_->get_logger(), "Publishing the MuJoCo model on '%s' (%ld bytes) and the state on '%s'.",
                config.model_topic.c_str(), static_cast<long>(size), config.state_topic.c_str());
}

VisualizationPublisher::~VisualizationPublisher()
{
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
}

void VisualizationPublisher::store_state(const mjData* d) noexcept
{
    std::unique_lock<std::mutex> lock(buffer_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;
    }
    buffer_[0] = d->time;
    std::copy(d->qpos, d->qpos + (buffer_.size() - 1), buffer_.begin() + 1);
    buffer_updated_ = true;
}

void VisualizationPublisher::publish_state()
{
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!buffer_updated_) {
            return;
        }
        state_msg_.data = buffer_;
        buffer_updated_ = false;
    }
    state_pub_->publish(state_msg_);
}

} // namespace mujoco_sim
} // namespace etasl
