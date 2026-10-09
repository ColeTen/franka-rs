# Reference programs that print libfranka's own outputs for the franka-rs validation tests to
# compare against. Each program writes a plain-text file under validation/data/.
#
# Configure and build:
#   cmake -S validation/reference -B validation/reference/build
#   cmake --build validation/reference/build
#
# message_layout needs only libfranka's common headers (no built library). The computation and
# model reference programs (added in their stages) link the built Franka package; point
# find_package at the libfranka build with -DFranka_DIR=<path-to>/external/libfranka/build.

cmake_minimum_required(VERSION 3.16)
project(franka_rs_validation_reference CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIBFRANKA ${CMAKE_CURRENT_SOURCE_DIR}/../../external/libfranka)

# Stage 1: wire-format sizes, field offsets, enum values, and protocol version. Headers only.
add_executable(message_layout message_layout.cpp)
target_include_directories(message_layout PRIVATE ${LIBFRANKA}/common/include)

# Stage 2: low-pass filter and rate-limiting outputs, computed by the built libfranka library
# itself (the code franka-rs is compared against, with its compiler flags).
find_package(Eigen3 REQUIRED)

# Stage 3: kinematic/dynamic model outputs. franka::Model is backed by Pinocchio, so this needs the
# full built libfranka package; point find_package at the libfranka build, e.g.
#   -DFranka_DIR=<repo>/external/libfranka/build
find_package(Franka REQUIRED)
add_executable(model model.cpp)
# The model reference also calls pinocchio directly, for the Coriolis matrix libfranka does not expose.
find_package(pinocchio REQUIRED)
target_link_libraries(model Franka::Franka pinocchio::pinocchio)

add_executable(computation computation.cpp)
target_link_libraries(computation Franka::Franka Eigen3::Eigen)

# communication_test with the target pose taken from the command line, for zero-torque hold checks
# at several poses. Uses the libfranka examples' MotionGenerator and setDefaultBehavior, built with
# libfranka's examples (BUILD_EXAMPLES=ON).
add_executable(pose_hold_test pose_hold_test.cpp)
target_include_directories(pose_hold_test PRIVATE ${LIBFRANKA}/examples)
target_link_libraries(pose_hold_test ${LIBFRANKA}/build/examples/libexamples_common.a Franka::Franka Eigen3::Eigen)

# Torque-control client for the A1 offline replay comparison (validation/a1/). Floating-point
# contraction is disabled so the arithmetic matches the Rust counterpart operation for operation.
add_executable(a1_torque_client a1_torque_client.cpp)
target_link_libraries(a1_torque_client Franka::Franka)
target_compile_options(a1_torque_client PRIVATE -ffp-contract=off)

# A1 motion-mode client: the same law per mode as examples/a1_motion_client.rs, for the offline
# bit-level comparison against the mock robot.
add_executable(a1_motion_client a1_motion_client.cpp)
target_link_libraries(a1_motion_client Franka::Franka)
target_compile_options(a1_motion_client PRIVATE -ffp-contract=off)

# Fixed command sequence for validation step A2 (torque and motion modes): sends commands read from
# a file on the real robot, for a bit-level comparison with examples/a2_fixed_sequence.rs.
add_executable(a2_fixed_sequence a2_fixed_sequence.cpp)
target_link_libraries(a2_fixed_sequence Franka::Franka)

# Torque-tracking measurement for validation step B: moves to a pose, holds it with joint impedance
# torque control, and logs commanded and measured torques. Uses the libfranka examples'
# MotionGenerator and setDefaultBehavior.
add_executable(impedance_hold_test impedance_hold_test.cpp)
target_include_directories(impedance_hold_test PRIVATE ${LIBFRANKA}/examples)
target_link_libraries(impedance_hold_test ${LIBFRANKA}/build/examples/libexamples_common.a Franka::Franka Eigen3::Eigen)
