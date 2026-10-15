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

/**
 * Test executable for robot simulator plugins of crospi (in particular mujoco_robotsimulator_crospi).
 *
 * It reads a crospi setup JSON file, selects one entry of blackboard/robot/robotsimulators, loads the corresponding
 * plugin through pluginlib (exactly as the crospi_node does), and drives it in a loop with the joint velocity profile
 *
 *     qd_i(t) = amplitude * sin(2 pi frequency t)     (same for every joint)
 *
 * whose ideal integral is q_i(t) = q0_i + amplitude / (2 pi frequency) * (1 - cos(2 pi frequency t)).
 * The simulator is updated every `periodicity` seconds (from the configuration). At the end, the tracking error
 * between the ideal and the simulated joint positions is reported, and all samples can be written to a CSV file.
 *
 * All arguments are required (there are no hidden parameters):
 *   --config <file>            crospi setup JSON file ($[package] paths are supported)
 *   --simulator-index <int>    index in blackboard/robot/robotsimulators
 *   --duration <s>             duration of the test (simulated time)
 *   --amplitude <rad/s>        amplitude of the velocity profile
 *   --frequency <Hz>           frequency of the velocity profile
 *   --real-time <true|false>   true: one update every periodicity seconds of wall time; false: as fast as possible
 *   --csv <file|none>          CSV output with time, setpoints, ideal positions and feedback, or none
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <jsoncpp/json/json.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "crospi_utils/json_checker.hpp"
#include "crospi_utils/string_interpolate.hpp"
#include "robot_interfacing_utils/robotdriver.hpp"

namespace {

const std::vector<std::string> REQUIRED_ARGS = {
    "--config", "--simulator-index", "--duration", "--amplitude", "--frequency", "--real-time", "--csv"};

void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " --config <setup.json> --simulator-index <int> --duration <s> --amplitude <rad/s>"
                 " --frequency <Hz> --real-time <true|false> --csv <file|none>\n";
}

std::map<std::string, std::string> parse_args(const std::vector<std::string>& args) {
    std::map<std::string, std::string> result;
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& key = args[i];
        if (std::find(REQUIRED_ARGS.begin(), REQUIRED_ARGS.end(), key) == REQUIRED_ARGS.end()) {
            throw std::runtime_error(fmt::format("Unknown argument '{}'.", key));
        }
        if (i + 1 >= args.size()) {
            throw std::runtime_error(fmt::format("Missing value for argument '{}'.", key));
        }
        result[key] = args[++i];
    }
    for (const auto& key : REQUIRED_ARGS) {
        if (!result.count(key)) {
            throw std::runtime_error(fmt::format("The argument '{}' is required.", key));
        }
    }
    return result;
}

bool parse_bool(const std::string& key, const std::string& value) {
    if (value == "true") return true;
    if (value == "false") return false;
    throw std::runtime_error(fmt::format("Argument '{}' must be true or false (got '{}').", key, value));
}

Json::Value read_json(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error(fmt::format("Could not open the configuration file '{}'.", path));
    }
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    if (!Json::parseFromStream(builder, file, &root, &errors)) {
        throw std::runtime_error(fmt::format("Could not parse '{}': {}", path, errors));
    }
    return root;
}

std::string simulator_type(const Json::Value& sim_config) {
    for (const auto& key : sim_config.getMemberNames()) {
        if (key.rfind("is-", 0) == 0) {
            return key.substr(3);
        }
    }
    throw std::runtime_error("The selected robotsimulator has no 'is-<plugin_name>' key.");
}

int run(const std::map<std::string, std::string>& args) {
    const std::string config_path = etasl::string_interpolate(args.at("--config"));
    const int sim_index = std::stoi(args.at("--simulator-index"));
    const double duration = std::stod(args.at("--duration"));
    const double amplitude = std::stod(args.at("--amplitude"));
    const double frequency = std::stod(args.at("--frequency"));
    const bool real_time = parse_bool("--real-time", args.at("--real-time"));
    const std::string csv_path = args.at("--csv");

    // ------------------------------------------------------------ configuration
    Json::Value root = read_json(config_path);
    const Json::Value& simulators = root["blackboard"]["robot"]["robotsimulators"];
    if (!simulators.isArray() || sim_index < 0 || sim_index >= static_cast<int>(simulators.size())) {
        throw std::runtime_error(fmt::format("'{}' has no blackboard/robot/robotsimulators[{}].", config_path, sim_index));
    }
    const Json::Value& sim_config = simulators[sim_index];
    const std::string plugin_name = simulator_type(sim_config);
    const double periodicity = sim_config["periodicity"].asDouble();

    auto jsonchecker = std::make_shared<etasl::JsonChecker>([](const std::string& msg) {
        throw std::runtime_error("JsonChecker: " + msg);
    });

    // ------------------------------------------------------------ load the plugin as crospi_node does
    pluginlib::ClassLoader<etasl::RobotDriver> loader("crospi_core", "etasl::RobotSimulator");
    std::shared_ptr<etasl::RobotDriver> sim = loader.createSharedInstance("etasl::" + plugin_name);

    sim->construct(plugin_name, sim_config, jsonchecker);
    if (!sim->initialize()) {
        throw std::runtime_error("initialize() returned false.");
    }
    sim->on_configure();
    sim->on_activate();

    robotdrivers::DynamicJointDataField q0;
    if (!sim->readFeedbackJointPosition(q0)) {
        throw std::runtime_error("No initial joint position feedback available.");
    }
    const std::size_t n = q0.data.size();

    std::cout << fmt::format("Loaded '{}' with {} joints. Available feedback: joint_pos={} joint_vel={} joint_torque={} "
                             "cartesian_pos={} cartesian_quat={} cartesian_twist={} cartesian_wrench={} base_pos={} "
                             "base_quat={} base_twist={}",
                             plugin_name, n, sim->isJointPosAvailable(), sim->isJointVelAvailable(),
                             sim->isJointTorqueAvailable(), sim->isCartesianPosAvailable(), sim->isCartesianQuatAvailable(),
                             sim->isCartesianTwistAvailable(), sim->isCartesianWrenchAvailable(), sim->isBasePosAvailable(),
                             sim->isBaseQuatAvailable(), sim->isBaseTwistAvailable())
              << std::endl;

    std::ofstream csv;
    if (csv_path != "none") {
        csv.open(csv_path);
        if (!csv) {
            throw std::runtime_error(fmt::format("Could not open '{}' for writing.", csv_path));
        }
        csv << "time";
        for (std::size_t i = 0; i < n; ++i) csv << ",qd_sp_" << i;
        for (std::size_t i = 0; i < n; ++i) csv << ",q_ideal_" << i;
        for (std::size_t i = 0; i < n; ++i) csv << ",q_" << i;
        if (sim->isJointTorqueAvailable()) for (std::size_t i = 0; i < n; ++i) csv << ",tau_" << i;
        if (sim->isCartesianWrenchAvailable()) csv << ",fx,fy,fz,tx,ty,tz";
        csv << "\n";
    }

    // ------------------------------------------------------------ control loop
    const double omega = 2.0 * M_PI * frequency;
    const long steps = std::lround(duration / periodicity);
    volatile std::atomic<bool> stop_flag{false};
    robotdrivers::DynamicJointDataField setpoint(static_cast<int>(n));
    robotdrivers::DynamicJointDataField q_fb(static_cast<int>(n));
    robotdrivers::DynamicJointDataField tau_fb(static_cast<int>(n));
    robotdrivers::ScrewField wrench{};
    std::vector<double> max_error(n, 0.0);

    auto next_wakeup = std::chrono::steady_clock::now();
    const auto period = std::chrono::nanoseconds(static_cast<long long>(periodicity * 1e9));
    const auto wall_start = std::chrono::steady_clock::now();

    for (long k = 0; k < steps && rclcpp::ok(); ++k) {
        // The setpoint applied during [t, t + periodicity] is evaluated at the middle of the interval,
        // so that its integral matches the ideal trajectory up to second order.
        const double t = k * periodicity;
        const double t_mid = t + 0.5 * periodicity;
        for (std::size_t i = 0; i < n; ++i) {
            setpoint.data[i] = static_cast<float>(amplitude * std::sin(omega * t_mid));
        }
        sim->writeSetpointJointVelocity(setpoint);
        sim->update(stop_flag);
        sim->readFeedbackJointPosition(q_fb);

        const double t_next = t + periodicity;
        const double q_offset = (frequency > 0.0) ? amplitude / omega * (1.0 - std::cos(omega * t_next)) : 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            max_error[i] = std::max(max_error[i], std::abs(q0.data[i] + q_offset - q_fb.data[i]));
        }

        if (csv.is_open()) {
            csv << t_next;
            for (std::size_t i = 0; i < n; ++i) csv << "," << setpoint.data[i];
            for (std::size_t i = 0; i < n; ++i) csv << "," << q0.data[i] + q_offset;
            for (std::size_t i = 0; i < n; ++i) csv << "," << q_fb.data[i];
            if (sim->isJointTorqueAvailable()) {
                sim->readFeedbackJointTorque(tau_fb);
                for (std::size_t i = 0; i < n; ++i) csv << "," << tau_fb.data[i];
            }
            if (sim->isCartesianWrenchAvailable()) {
                sim->readFeedbackCartesianWrench(wrench);
                csv << "," << wrench.linear.x << "," << wrench.linear.y << "," << wrench.linear.z << ","
                    << wrench.angular.x << "," << wrench.angular.y << "," << wrench.angular.z;
            }
            csv << "\n";
        }

        if (real_time) {
            next_wakeup += period;
            std::this_thread::sleep_until(next_wakeup);
        }
    }
    const double wall_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();

    // ------------------------------------------------------------ report
    std::cout << fmt::format("Simulated {:.3f} s in {:.3f} s of wall time (real-time factor {:.2f}).",
                             steps * periodicity, wall_time, steps * periodicity / wall_time) << std::endl;
    std::cout << "Final joint positions [ideal | simulated] and max tracking error:" << std::endl;
    const double q_offset_end = (frequency > 0.0) ? amplitude / omega * (1.0 - std::cos(omega * steps * periodicity)) : 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        std::cout << fmt::format("  joint {}: {:+.5f} | {:+.5f}   max error {:.2e}", i, q0.data[i] + q_offset_end,
                                 q_fb.data[i], max_error[i]) << std::endl;
    }
    if (sim->isCartesianPosAvailable() && sim->isCartesianQuatAvailable()) {
        robotdrivers::Vector3Field p{};
        robotdrivers::QuaternionField q{};
        sim->readFeedbackCartesianPosition(p);
        sim->readFeedbackCartesianQuaternion(q);
        std::cout << fmt::format("  tcp pose w.r.t. base: p=[{:+.4f} {:+.4f} {:+.4f}] q(xyzw)=[{:+.4f} {:+.4f} {:+.4f} {:+.4f}]",
                                 p.x, p.y, p.z, q.qx, q.qy, q.qz, q.qw) << std::endl;
    }
    if (sim->isCartesianWrenchAvailable()) {
        sim->readFeedbackCartesianWrench(wrench);
        std::cout << fmt::format("  tcp wrench: f=[{:+.3f} {:+.3f} {:+.3f}] t=[{:+.3f} {:+.3f} {:+.3f}]",
                                 wrench.linear.x, wrench.linear.y, wrench.linear.z,
                                 wrench.angular.x, wrench.angular.y, wrench.angular.z) << std::endl;
    }

    sim->on_deactivate();
    sim->on_cleanup();
    sim->finalize();
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    // ROS arguments (--ros-args ...) are removed before parsing the arguments of this executable.
    rclcpp::init(argc, argv);
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    args.erase(args.begin());

    int result = 1;
    try {
        result = run(parse_args(args));
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        print_usage(argv[0]);
    }
    rclcpp::shutdown();
    return result;
}
