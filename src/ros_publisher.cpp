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

#include "mujoco_robotsimulator_crospi/ros_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include <fmt/format.h>

namespace etasl {
namespace mujoco_sim {

namespace {

const std::string PLUGIN_NAME = "mujoco_robotsimulator_crospi";

void fill_pose(geometry_msgs::msg::Pose& pose, const std::array<double, 3>& p, const std::array<double, 4>& q_wxyz) {
    pose.position.x = p[0];
    pose.position.y = p[1];
    pose.position.z = p[2];
    pose.orientation.w = q_wxyz[0];
    pose.orientation.x = q_wxyz[1];
    pose.orientation.y = q_wxyz[2];
    pose.orientation.z = q_wxyz[3];
}

void fill_vector(geometry_msgs::msg::Vector3& v, const std::array<double, 3>& a) {
    v.x = a[0];
    v.y = a[1];
    v.z = a[2];
}

} // namespace


RosPublisher::RosPublisher(const SimulatorConfig& config, const mjModel* m)
    : pub_config_(config.sensor_publishing)
    , base_frame_(config.base_link.value_or(""))
    , tcp_frame_(config.tcp_link.value_or(""))
    , model_(m)
{
    if (!rclcpp::ok()) {
        throw std::runtime_error(fmt::format("[{}] The simulator needs an initialized ROS 2 context (rclcpp::init) "
                                             "to publish the sensors.", PLUGIN_NAME));
    }

    // ---------------------------------------------------------------- MuJoCo sensors to publish
    std::vector<int> sensor_ids;
    if (pub_config_.all_mujoco_sensors) {
        for (int i = 0; i < m->nsensor; ++i) {
            const char* sensor_name = mj_id2name(m, mjOBJ_SENSOR, i);
            if (sensor_name) {  // unnamed sensors cannot be published
                sensor_ids.push_back(i);
            }
        }
    } else {
        for (const auto& sensor_name : pub_config_.mujoco_sensors) {
            int id = mj_name2id(m, mjOBJ_SENSOR, sensor_name.c_str());
            if (id < 0) {
                std::string available = fmt::format("{}, {}, {}, {}, {}, {}", SENSOR_JOINT_STATE, SENSOR_TCP_POSE,
                                                    SENSOR_TCP_TWIST, SENSOR_TCP_WRENCH, SENSOR_BASE_POSE, SENSOR_BASE_TWIST);
                for (int i = 0; i < m->nsensor; ++i) {
                    const char* n = mj_id2name(m, mjOBJ_SENSOR, i);
                    if (n) {
                        available += fmt::format(", {}", n);
                    }
                }
                throw std::runtime_error(fmt::format("[{}] Invalid configuration: unknown sensor '{}' in "
                                                     "'sensor_publishing/sensors'. Available sensors: {}.",
                                                     PLUGIN_NAME, sensor_name, available));
            }
            sensor_ids.push_back(id);
        }
    }

    // ---------------------------------------------------------------- node and publishers
    node_ = std::make_shared<rclcpp::Node>(config.ros_node_name);
    const auto qos = rclcpp::QoS(10);  // reliable: compatible with every subscriber (PlotJuggler, ros2 topic echo, ...)

    try {
        if (pub_config_.joint_state) {
            joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic("joint_states"), qos);
            joint_state_msg_.name = config.robot_joints;
            const std::size_t n = config.robot_joints.size();
            joint_state_msg_.position.assign(n, 0.0);
            joint_state_msg_.velocity.assign(n, 0.0);
            joint_state_msg_.effort.assign(n, 0.0);
        }
        if (pub_config_.tcp_pose) {
            tcp_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic("tcp_pose"), qos);
        }
        if (pub_config_.tcp_twist) {
            tcp_twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic("tcp_twist"), qos);
        }
        if (pub_config_.tcp_wrench) {
            tcp_wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(topic("tcp_wrench"), qos);
        }
        if (pub_config_.base_pose) {
            base_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic("base_pose"), qos);
        }
        if (pub_config_.base_twist) {
            base_twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic("base_twist"), qos);
        }
        for (int id : sensor_ids) {
            const std::string sensor_name = mj_id2name(m, mjOBJ_SENSOR, id);
            MujocoSensorPublisher sp;
            sp.adr = m->sensor_adr[id];
            sp.dim = m->sensor_dim[id];
            sp.pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic("mujoco_sensors/" + sensor_name), qos);
            sp.msg.data.assign(sp.dim, 0.0);
            std_msgs::msg::MultiArrayDimension dim;
            dim.label = sensor_name;
            dim.size = sp.dim;
            dim.stride = sp.dim;
            sp.msg.layout.dim.push_back(dim);
            mj_sensor_pubs_.push_back(sp);
            mj_sensor_names_.push_back(sensor_name);
        }
    } catch (const std::exception& e) {
        throw std::runtime_error(fmt::format("[{}] Could not create the sensor publishers (check topic_namespace and "
                                             "the names of the MuJoCo sensors): {}", PLUGIN_NAME, e.what()));
    }

    const std::size_t n = config.robot_joints.size();
    for (Measurements* meas : {&meas_buffer_, &meas_copy_}) {
        meas->q.assign(n, 0.0);
        meas->qd.assign(n, 0.0);
        meas->tau.assign(n, 0.0);
    }
    sensordata_buffer_.assign(m->nsensordata, 0.0);
    sensordata_copy_.assign(m->nsensordata, 0.0);

    sensor_timer_ = node_->create_wall_timer(std::chrono::duration<double>(pub_config_.publish_period),
                                             [this]() { publish_sensors(); });

    // ---------------------------------------------------------------- visualization
    if (config.visualization) {
        const VisualizationConfig& vis = *config.visualization;
        model_pub_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(vis.model_topic,
                                                                             rclcpp::QoS(1).reliable().transient_local());
        std_msgs::msg::UInt8MultiArray model_msg;
        mjtSize size = mj_sizeModel(m);
        model_msg.data.resize(static_cast<std::size_t>(size));
        mj_saveModel(m, nullptr, model_msg.data.data(), size);
        model_pub_->publish(model_msg);

        state_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(vis.state_topic, rclcpp::QoS(1));
        qpos_buffer_.assign(1 + m->nq, 0.0);
        state_msg_.data.assign(1 + m->nq, 0.0);
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "time_and_qpos";
        dim.size = 1 + m->nq;
        dim.stride = 1 + m->nq;
        state_msg_.layout.dim.push_back(dim);
        if (vis.contact_topic) {
            contact_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(*vis.contact_topic, rclcpp::QoS(1));
            contacts_buffer_.assign(max_contacts(m) * CONTACT_STRIDE, 0.0);
            std_msgs::msg::MultiArrayDimension dim;
            dim.label = "time_ncontacts_then_body_otherbody_px_py_pz_fx_fy_fz";
            dim.stride = CONTACT_STRIDE;
            contact_msg_.layout.dim.push_back(dim);
            RCLCPP_INFO(node_->get_logger(), "Publishing the contact forces on '%s'.", vis.contact_topic->c_str());
        }
        visualization_timer_ = node_->create_wall_timer(std::chrono::duration<double>(vis.publish_period),
                                                        [this]() { publish_visualization_state(); });
        RCLCPP_INFO(node_->get_logger(), "Publishing the MuJoCo model on '%s' (%ld bytes) and the state on '%s'.",
                    vis.model_topic.c_str(), static_cast<long>(size), vis.state_topic.c_str());
    }

    // ---------------------------------------------------------------- external wrench
    if (config.external_wrench) {
        try {
            external_wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
                config.external_wrench->topic, rclcpp::QoS(10),
                [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { on_external_wrench(msg); });
        } catch (const std::exception& e) {
            throw std::runtime_error(fmt::format("[{}] Could not subscribe to the external wrench topic '{}': {}",
                                                 PLUGIN_NAME, config.external_wrench->topic, e.what()));
        }
        RCLCPP_INFO(node_->get_logger(), "Receiving external wrenches on '%s' (timeout %g s, max. %g N, %g Nm).",
                    config.external_wrench->topic.c_str(), config.external_wrench->timeout,
                    config.external_wrench->max_force, config.external_wrench->max_torque);
    }

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    std::string published;
    for (const auto& [enabled, sensor_name] : std::vector<std::pair<bool, const char*>>{
             {pub_config_.joint_state, SENSOR_JOINT_STATE}, {pub_config_.tcp_pose, SENSOR_TCP_POSE},
             {pub_config_.tcp_twist, SENSOR_TCP_TWIST}, {pub_config_.tcp_wrench, SENSOR_TCP_WRENCH},
             {pub_config_.base_pose, SENSOR_BASE_POSE}, {pub_config_.base_twist, SENSOR_BASE_TWIST}}) {
        if (enabled) {
            published += std::string(published.empty() ? "" : ", ") + sensor_name;
        }
    }
    for (const auto& sensor_name : mj_sensor_names_) {
        published += std::string(published.empty() ? "" : ", ") + "mujoco_sensors/" + sensor_name;
    }
    RCLCPP_INFO(node_->get_logger(), "Publishing the sensors [%s] under '%s' every %g s.", published.c_str(),
                pub_config_.topic_namespace.c_str(), pub_config_.publish_period);
}

RosPublisher::~RosPublisher()
{
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
}

std::string RosPublisher::topic(const std::string& relative) const
{
    std::string ns = pub_config_.topic_namespace;
    while (!ns.empty() && ns.back() == '/') {
        ns.pop_back();
    }
    return ns.empty() ? relative : ns + "/" + relative;
}

void RosPublisher::store(const mjData* d, const Measurements& meas, const std::vector<double>& contacts,
                         std::size_t n_contacts) noexcept
{
    std::unique_lock<std::mutex> lock(buffer_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;
    }
    meas_buffer_ = meas;  // same sizes: no allocation
    std::copy(d->sensordata, d->sensordata + sensordata_buffer_.size(), sensordata_buffer_.begin());
    sensors_updated_ = true;
    if (!qpos_buffer_.empty()) {
        qpos_buffer_[0] = d->time;
        std::copy(d->qpos, d->qpos + (qpos_buffer_.size() - 1), qpos_buffer_.begin() + 1);
        if (contact_pub_) {
            contacts_count_ = std::min(n_contacts, contacts_buffer_.size() / CONTACT_STRIDE);
            std::copy(contacts.begin(), contacts.begin() + contacts_count_ * CONTACT_STRIDE, contacts_buffer_.begin());
        }
        state_updated_ = true;
    }
}

void RosPublisher::on_external_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    const int body = mj_name2id(model_, mjOBJ_BODY, msg->header.frame_id.c_str());
    if (body <= 0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "External wrench ignored: '%s' (header.frame_id) is not a body of the simulation.",
                             msg->header.frame_id.c_str());
        return;
    }
    ExternalWrench w;
    w.body = body;
    w.force = {msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z};
    w.torque = {msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z};
    w.received = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(external_wrench_mutex_);
    external_wrench_buffer_ = w;
    external_wrench_updated_ = true;
}

bool RosPublisher::fetch_external_wrench(ExternalWrench& wrench) noexcept
{
    std::unique_lock<std::mutex> lock(external_wrench_mutex_, std::try_to_lock);
    if (!lock.owns_lock() || !external_wrench_updated_) {
        return false;
    }
    wrench = external_wrench_buffer_;
    external_wrench_updated_ = false;
    return true;
}

void RosPublisher::publish_sensors()
{
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!sensors_updated_) {
            return;
        }
        meas_copy_ = meas_buffer_;
        sensordata_copy_ = sensordata_buffer_;
        sensors_updated_ = false;
    }
    const auto stamp = node_->now();
    const Measurements& meas = meas_copy_;

    if (joint_state_pub_) {
        joint_state_msg_.header.stamp = stamp;
        joint_state_msg_.position = meas.q;
        joint_state_msg_.velocity = meas.qd;
        joint_state_msg_.effort = meas.tau;
        joint_state_pub_->publish(joint_state_msg_);
    }
    if (tcp_pose_pub_) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = base_frame_;
        fill_pose(msg.pose, meas.tcp_pos, meas.tcp_quat);
        tcp_pose_pub_->publish(msg);
    }
    if (tcp_twist_pub_) {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = base_frame_;
        fill_vector(msg.twist.linear, meas.tcp_lin_vel);
        fill_vector(msg.twist.angular, meas.tcp_ang_vel);
        tcp_twist_pub_->publish(msg);
    }
    if (tcp_wrench_pub_) {
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = tcp_frame_;
        fill_vector(msg.wrench.force, meas.tcp_force);
        fill_vector(msg.wrench.torque, meas.tcp_torque);
        tcp_wrench_pub_->publish(msg);
    }
    if (base_pose_pub_) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "world";
        fill_pose(msg.pose, meas.base_pos, meas.base_quat);
        base_pose_pub_->publish(msg);
    }
    if (base_twist_pub_) {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "world";
        fill_vector(msg.twist.linear, meas.base_lin_vel);
        fill_vector(msg.twist.angular, meas.base_ang_vel);
        base_twist_pub_->publish(msg);
    }
    for (auto& sp : mj_sensor_pubs_) {
        std::copy(sensordata_copy_.begin() + sp.adr, sensordata_copy_.begin() + sp.adr + sp.dim, sp.msg.data.begin());
        sp.pub->publish(sp.msg);
    }
}

void RosPublisher::publish_visualization_state()
{
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!state_updated_) {
            return;
        }
        state_msg_.data = qpos_buffer_;
        if (contact_pub_) {
            contact_msg_.data.resize(2 + contacts_count_ * CONTACT_STRIDE);
            contact_msg_.data[0] = qpos_buffer_[0];
            contact_msg_.data[1] = static_cast<double>(contacts_count_);
            std::copy(contacts_buffer_.begin(), contacts_buffer_.begin() + contacts_count_ * CONTACT_STRIDE,
                      contact_msg_.data.begin() + 2);
            contact_msg_.layout.dim[0].size = contacts_count_;
        }
        state_updated_ = false;
    }
    state_pub_->publish(state_msg_);
    if (contact_pub_) {
        contact_pub_->publish(contact_msg_);
    }
}

} // namespace mujoco_sim
} // namespace etasl
