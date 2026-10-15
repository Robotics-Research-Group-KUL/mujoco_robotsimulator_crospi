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

#include "mujoco_robotsimulator_crospi/mujoco_scene_builder.hpp"

#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>

#include <fmt/format.h>

namespace etasl {
namespace mujoco_sim {

namespace {

const std::string PLUGIN_NAME = "mujoco_robotsimulator_crospi";

[[noreturn]] void fail(const std::string& msg) {
    throw std::runtime_error(fmt::format("[{}] Could not build the MuJoCo model: {}", PLUGIN_NAME, msg));
}

struct SpecDeleter {
    void operator()(mjSpec* s) const { if (s) mj_deleteSpec(s); }
};
using SpecPtr = std::unique_ptr<mjSpec, SpecDeleter>;

SpecPtr parse_file(const std::string& path, const std::string& what) {
    char error[1024] = "";
    mjSpec* spec = mj_parseXML(path.c_str(), nullptr, error, sizeof(error));
    if (!spec) {
        fail(fmt::format("parsing the {} '{}' failed: {}", what, path, error));
    }
    return SpecPtr(spec);
}

/** Resolves package://, file:// and relative mesh paths of the URDF to absolute paths. */
void resolve_mesh_paths(mjSpec* robot, const std::string& urdf_dir) {
    for (mjsElement* el = mjs_firstElement(robot, mjOBJ_MESH); el; el = mjs_nextElement(robot, el)) {
        mjsMesh* mesh = mjs_asMesh(el);
        std::string file = mjs_getString(mesh->file);
        if (file.empty()) {
            continue;
        }
        std::string resolved = resolve_path(file, urdf_dir);
        if (!std::filesystem::is_regular_file(resolved)) {
            fail(fmt::format("the mesh '{}' of the URDF resolves to '{}', which does not exist.", file, resolved));
        }
        mjs_setString(mesh->file, resolved.c_str());
    }
}

void apply_joint_overrides(mjSpec* robot, const SimulatorConfig& config) {
    const auto& dyn = config.joint_dynamics;
    for (std::size_t i = 0; i < config.robot_joints.size(); ++i) {
        const std::string& name = config.robot_joints[i];
        mjsElement* el = mjs_findElement(robot, mjOBJ_JOINT, name.c_str());
        if (!el) {
            fail(fmt::format("the joint '{}' listed in robot_joints does not exist in the URDF '{}'.", name, config.urdf_path));
        }
        mjsJoint* joint = mjs_asJoint(el);
        if (!dyn.armature.empty()) {
            joint->armature = dyn.armature[i];
        }
        if (!dyn.damping.empty()) {
            joint->damping[0] = dyn.damping[i];
        }
        if (!dyn.frictionloss.empty()) {
            joint->frictionloss = dyn.frictionloss[i];
        }
    }
}

std::set<std::string> body_names(mjSpec* spec) {
    std::set<std::string> names;
    for (mjsElement* el = mjs_firstElement(spec, mjOBJ_BODY); el; el = mjs_nextElement(spec, el)) {
        std::string body_name = mjs_getString(mjs_getName(el));
        if (!body_name.empty() && body_name != "world") {
            names.insert(body_name);
        }
    }
    return names;
}

/**
 * Excludes the contacts between adjacent links of the robot, as in the URDF convention (and in other simulators).
 *
 * MuJoCo groups bodies that are welded together (no joint between them) and filters contacts between a body and its
 * parent, except when the parent is welded to the world. Hence, the first moving link of a robot fixed to the world
 * (e.g. shoulder_link of a UR robot) would collide with the base link, whose meshes overlap at the joint.
 * Here, the contacts between all robot bodies of a weld group and all robot bodies of its parent weld group are
 * excluded explicitly.
 */
void exclude_adjacent_link_contacts(mjSpec* scene, const mjModel* m, const std::set<std::string>& robot_bodies) {
    auto is_robot_body_with_geoms = [&](int b) {
        return m->body_geomnum[b] > 0 && robot_bodies.count(mj_id2name(m, mjOBJ_BODY, b));
    };
    for (int child_root = 1; child_root < m->nbody; ++child_root) {
        // first body of a moving weld group of the robot
        if (m->body_weldid[child_root] != child_root || !robot_bodies.count(mj_id2name(m, mjOBJ_BODY, child_root))) {
            continue;
        }
        const int parent_group = m->body_weldid[m->body_parentid[child_root]];
        for (int x = 1; x < m->nbody; ++x) {
            if (m->body_weldid[x] != child_root || !is_robot_body_with_geoms(x)) {
                continue;
            }
            for (int y = 1; y < m->nbody; ++y) {
                if (m->body_weldid[y] != parent_group || !is_robot_body_with_geoms(y)) {
                    continue;
                }
                mjsExclude* exclude = mjs_addExclude(scene);
                mjs_setString(exclude->bodyname1, mj_id2name(m, mjOBJ_BODY, x));
                mjs_setString(exclude->bodyname2, mj_id2name(m, mjOBJ_BODY, y));
            }
        }
    }
}

int integrator_enum(const std::string& name) {
    if (name == "Euler") return mjINT_EULER;
    if (name == "RK4") return mjINT_RK4;
    if (name == "implicit") return mjINT_IMPLICIT;
    return mjINT_IMPLICITFAST;
}

} // namespace


mjModel* build_model(const SimulatorConfig& config)
{
    // ------------------------------------------------------------------ robot (URDF)
    SpecPtr robot = parse_file(config.urdf_path, "URDF");
    robot->strippath = 0;                // keep the (resolved) mesh paths
    robot->compiler.fusestatic = 0;      // keep links attached with fixed joints (e.g. tool0) as bodies
    resolve_mesh_paths(robot.get(), std::filesystem::path(config.urdf_path).parent_path().string());
    apply_joint_overrides(robot.get(), config);
    const std::set<std::string> robot_bodies = body_names(robot.get());

    if (config.gravity_compensation) {
        for (mjsElement* el = mjs_firstElement(robot.get(), mjOBJ_BODY); el; el = mjs_nextElement(robot.get(), el)) {
            mjs_asBody(el)->gravcomp = 1.0;
        }
    }

    // ------------------------------------------------------------------ scene (environment MJCF or empty world)
    SpecPtr scene = config.environment_mjcf_path ? parse_file(*config.environment_mjcf_path, "environment MJCF")
                                                 : SpecPtr(mj_makeSpec());
    scene->strippath = 0;
    scene->compiler.fusestatic = 0;

    scene->option.timestep = config.simulation_timestep;
    scene->option.integrator = integrator_enum(config.integrator);
    for (int i = 0; i < 3; ++i) {
        scene->option.gravity[i] = config.gravity[i];
    }
    if (config.contacts_enabled) {
        scene->option.disableflags &= ~mjDSBL_CONTACT;
    } else {
        scene->option.disableflags |= mjDSBL_CONTACT;
    }

    mjsBody* world = mjs_findBody(scene.get(), "world");
    if (!world) {
        fail("the world body of the scene could not be found.");
    }

    if (config.add_ground_plane) {
        mjsGeom* plane = mjs_addGeom(world, nullptr);
        mjs_setName(plane->element, GROUND_PLANE_NAME);
        plane->type = mjGEOM_PLANE;
        plane->size[0] = 0.0;  // infinite plane
        plane->size[1] = 0.0;
        plane->size[2] = 0.05; // spacing of the rendered grid
        plane->rgba[0] = 0.6f;
        plane->rgba[1] = 0.6f;
        plane->rgba[2] = 0.6f;
        plane->rgba[3] = 1.0f;
    }

    // ------------------------------------------------------------------ attach the robot at the base pose
    mjsFrame* base_frame = mjs_addFrame(world, nullptr);
    for (int i = 0; i < 3; ++i) {
        base_frame->pos[i] = config.base_position[i];
    }
    // MuJoCo quaternions are [w x y z]
    base_frame->quat[0] = config.base_quaternion_xyzw[3];
    base_frame->quat[1] = config.base_quaternion_xyzw[0];
    base_frame->quat[2] = config.base_quaternion_xyzw[1];
    base_frame->quat[3] = config.base_quaternion_xyzw[2];

    if (!mjs_attach(base_frame->element, robot->element, "", "")) {
        fail(fmt::format("attaching the robot to the scene failed: {}", mjs_getError(scene.get())));
    }

    // ------------------------------------------------------------------ frames used for feedback
    for (const auto& link : {config.base_link, config.tcp_link}) {
        if (link && !mjs_findBody(scene.get(), link->c_str())) {
            fail(fmt::format("the link '{}' (base_link/tcp_link) does not exist in the URDF '{}'.", *link, config.urdf_path));
        }
    }

    // ------------------------------------------------------------------ force/torque sensor at the tcp
    if (config.feedback.cartesian_wrench) {
        mjsBody* tcp = mjs_findBody(scene.get(), config.tcp_link->c_str());
        mjsSite* site = mjs_addSite(tcp, nullptr);
        mjs_setName(site->element, FT_SITE_NAME);
        site->size[0] = 0.005;
        site->group = 4;  // hidden by default in the viewer

        mjsSensor* force = mjs_addSensor(scene.get());
        mjs_setName(force->element, FT_FORCE_SENSOR_NAME);
        force->type = mjSENS_FORCE;
        force->objtype = mjOBJ_SITE;
        mjs_setString(force->objname, FT_SITE_NAME);

        mjsSensor* torque = mjs_addSensor(scene.get());
        mjs_setName(torque->element, FT_TORQUE_SENSOR_NAME);
        torque->type = mjSENS_TORQUE;
        torque->objtype = mjOBJ_SITE;
        mjs_setString(torque->objname, FT_SITE_NAME);
    }

    // ------------------------------------------------------------------ compile
    // A first compilation provides the weld groups needed to exclude the contacts between adjacent robot links.
    mjModel* m = mj_compile(scene.get(), nullptr);
    if (!m) {
        fail(fmt::format("compilation failed: {}", mjs_getError(scene.get())));
    }
    exclude_adjacent_link_contacts(scene.get(), m, robot_bodies);
    mj_deleteModel(m);

    m = mj_compile(scene.get(), nullptr);
    if (!m) {
        fail(fmt::format("compilation failed: {}", mjs_getError(scene.get())));
    }
    return m;
}

} // namespace mujoco_sim
} // namespace etasl
