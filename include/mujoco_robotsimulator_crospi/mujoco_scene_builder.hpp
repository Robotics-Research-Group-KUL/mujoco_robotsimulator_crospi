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

#include <string>

#include <mujoco/mujoco.h>

#include "mujoco_robotsimulator_crospi/mujoco_simulator_config.hpp"

namespace etasl {
namespace mujoco_sim {

/** Name of the site added at tcp_link to measure the wrench with MuJoCo force/torque sensors. */
inline const char* FT_SITE_NAME = "crospi_ft_site";
inline const char* FT_FORCE_SENSOR_NAME = "crospi_ft_force";
inline const char* FT_TORQUE_SENSOR_NAME = "crospi_ft_torque";
inline const char* GROUND_PLANE_NAME = "crospi_ground_plane";

/**
 * @brief Builds the MuJoCo model described by the configuration:
 *   1. the URDF is parsed (package:// and relative mesh paths are resolved, fixed links are kept as bodies),
 *   2. the joint dynamics overrides and gravity compensation are applied to the robot,
 *   3. the robot is attached to the world of the environment MJCF (or an empty world) at the base pose,
 *   4. the ground plane, physics options and force/torque sensors are added,
 *   5. the scene is compiled.
 *
 * @return the compiled model, owned by the caller (free with mj_deleteModel).
 * @throws std::runtime_error if any of the steps fails.
 */
mjModel* build_model(const SimulatorConfig& config);

} // namespace mujoco_sim
} // namespace etasl
