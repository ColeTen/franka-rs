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

# Stage 2: low-pass filter and rate-limiting outputs. Compiles the two libfranka source files it
# exercises directly, so it needs only Eigen (no built library, no Pinocchio).
find_package(Eigen3 REQUIRED)
add_executable(computation computation.cpp
    ${LIBFRANKA}/src/lowpass_filter.cpp
    ${LIBFRANKA}/src/rate_limiting.cpp)
target_include_directories(computation PRIVATE ${LIBFRANKA}/include)
target_link_libraries(computation Eigen3::Eigen)

# Stage 3: kinematic/dynamic model outputs. franka::Model is backed by Pinocchio, so this needs the
# full built libfranka package; point find_package at the libfranka build, e.g.
#   -DFranka_DIR=<repo>/external/libfranka/build
find_package(Franka REQUIRED)
add_executable(model model.cpp)
target_link_libraries(model Franka::Franka)
