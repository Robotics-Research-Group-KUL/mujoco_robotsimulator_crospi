cmake_minimum_required(VERSION 3.8)
project(mujoco_robotsimulator_crospi)

message("--------------mujoco_robotsimulator_crospi compiling ----------------")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(crospi_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Required to resolve the link interface of crospi_core::crospi_utils (string_interpolate and JsonChecker)
find_package(crospi_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(etasl REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost 1.89 CONFIG REQUIRED)
message(STATUS "Using Boost version ${Boost_VERSION}")
message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")


# ----------------------------------------------------------------------------------------------
# MuJoCo
# The C API of MuJoCo (>= 3.3, with mjSpec) is located in the following order:
#   1. A CMake package (e.g. MuJoCo built from source and installed): find_package(mujoco CONFIG)
#   2. An explicit installation directory given by -DMUJOCO_DIR=<dir> or the environment variable
#      MUJOCO_DIR. The directory must contain include/mujoco/mujoco.h and lib/libmujoco.so* (release
#      tarball layout) or libmujoco.so* directly (pip wheel layout).
#   3. The MuJoCo python package (pip install mujoco), which ships the headers and the shared library.
# ----------------------------------------------------------------------------------------------
if(NOT DEFINED MUJOCO_DIR AND DEFINED ENV{MUJOCO_DIR})
  set(MUJOCO_DIR $ENV{MUJOCO_DIR})
endif()

if(NOT DEFINED MUJOCO_DIR)
  find_package(mujoco CONFIG QUIET)
endif()

if(TARGET mujoco::mujoco)
  message(STATUS "MuJoCo found as CMake package: ${mujoco_DIR}")
else()
  if(NOT DEFINED MUJOCO_DIR)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
      OUTPUT_VARIABLE MUJOCO_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _mujoco_python_result
      ERROR_QUIET)
    if(NOT _mujoco_python_result EQUAL 0)
      message(FATAL_ERROR "MuJoCo could not be found. Either install MuJoCo as a CMake package, set -DMUJOCO_DIR=<dir> (or the environment variable MUJOCO_DIR), or install the python package (pip install mujoco).")
    endif()
    message(STATUS "MuJoCo located through its python package")
  endif()

  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS ${MUJOCO_DIR}/include NO_DEFAULT_PATH)
  file(GLOB _mujoco_libs ${MUJOCO_DIR}/libmujoco.so* ${MUJOCO_DIR}/lib/libmujoco.so*)
  if(NOT MUJOCO_INCLUDE_DIR OR NOT _mujoco_libs)
    message(FATAL_ERROR "MuJoCo headers or library not found in MUJOCO_DIR=${MUJOCO_DIR}")
  endif()
  list(SORT _mujoco_libs)
  list(GET _mujoco_libs 0 MUJOCO_LIBRARY)

  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION ${MUJOCO_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_INCLUDE_DIR})
  get_filename_component(MUJOCO_LIBRARY_DIR ${MUJOCO_LIBRARY} DIRECTORY)
  message(STATUS "MuJoCo include dir: ${MUJOCO_INCLUDE_DIR}")
  message(STATUS "MuJoCo library: ${MUJOCO_LIBRARY}")
endif()

# The MuJoCo library is usually not in a system path, so the run-time path is embedded in the binaries.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
if(DEFINED MUJOCO_LIBRARY_DIR)
  list(APPEND CMAKE_INSTALL_RPATH ${MUJOCO_LIBRARY_DIR})
  list(APPEND CMAKE_BUILD_RPATH ${MUJOCO_LIBRARY_DIR})
endif()


# ----------------------------------------------------------------------------------------------
# Plugin library
# ----------------------------------------------------------------------------------------------
set(src_mujoco_robotsimulator_crospi
    src/mujoco_robotsimulator_crospi.cpp
    src/mujoco_simulator_config.cpp
    src/mujoco_scene_builder.cpp
    src/ros_publisher.cpp
)

add_library(mujoco_robotsimulator_crospi
            SHARED
            ${src_mujoco_robotsimulator_crospi})


# All commented dependencies are already inherited and linked thanks to crospi_core::robot_interfacing_utils (thanks to ament_export_dependencies)
set(rosdependencies
  pluginlib
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

set(dependencies
    crospi_core::robot_interfacing_utils
    crospi_core::crospi_utils   # string_interpolate ($[package] paths) and JsonChecker
    mujoco::mujoco
  )

set(ros_targets
    pluginlib::pluginlib
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ament_index_cpp::ament_index_cpp
  )


pluginlib_export_plugin_description_file(crospi_core plugins.xml)


target_include_directories(mujoco_robotsimulator_crospi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})


# ----------------------------------------------------------------------------------------------
# Test executable: loads the simulator plugin through pluginlib (exactly as the crospi_node does)
# and drives it with a velocity profile, without needing the full crospi_node.
# ----------------------------------------------------------------------------------------------
add_executable(test_mujoco_robotsimulator src/test_mujoco_robotsimulator.cpp)
target_include_directories(test_mujoco_robotsimulator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Boost_INCLUDE_DIRS})


#The following is done because ROS2 deprecated ament_target_dependencies in favor of target_link_libraries, but the former is still required for older (legacy) ROS2 versions.
if(DEFINED ENV{ROS_DISTRO} AND (
  "$ENV{ROS_DISTRO}" STREQUAL "foxy" OR
  "$ENV{ROS_DISTRO}" STREQUAL "galactic" OR
  "$ENV{ROS_DISTRO}" STREQUAL "humble" OR
  "$ENV{ROS_DISTRO}" STREQUAL "iron" OR
  "$ENV{ROS_DISTRO}" STREQUAL "jazzy"
))
      ament_target_dependencies(mujoco_robotsimulator_crospi ${rosdependencies})
      target_link_libraries(mujoco_robotsimulator_crospi ${dependencies})
      ament_target_dependencies(test_mujoco_robotsimulator ${rosdependencies})
      target_link_libraries(test_mujoco_robotsimulator crospi_core::robot_interfacing_utils crospi_core::crospi_utils)
      message(STATUS "Legacy ROS_DISTRO is $ENV{ROS_DISTRO}, using ament_target_dependencies")
else()
    target_link_libraries(mujoco_robotsimulator_crospi  ${dependencies} ${ros_targets})
    target_link_libraries(test_mujoco_robotsimulator crospi_core::robot_interfacing_utils crospi_core::crospi_utils ${ros_targets})
    message(STATUS "Modern ROS_DISTRO detected ($ENV{ROS_DISTRO}), using target_link_libraries")
endif()


ament_export_dependencies(crospi_core)

ament_export_libraries(
  mujoco_robotsimulator_crospi
)

install(
  TARGETS mujoco_robotsimulator_crospi
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS test_mujoco_robotsimulator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/mujoco_viewer.py scripts/run_crospi_tasks.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY json_schemas config betfsm_tasks_examples
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
