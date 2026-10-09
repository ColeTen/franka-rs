// Runs libfranka's kinematic/dynamic model on chosen configurations and prints each case's inputs
// and libfranka's outputs, for comparison against the franka-rs model in source/model/. Both sides
// are built from the same URDF, so this compares the two implementations' math directly.
//
// Unlike the stage-1 and stage-2 reference programs, this one needs the full libfranka library
// (franka::Model is backed by Pinocchio), so it is built with the libfranka CMake package inside
// the devcontainer rather than natively. Build it via validation/reference/CMakeLists.txt, then:
//
//   ./model <path-to-urdf> > validation/data/model_cases.txt
//
// where <path-to-urdf> is the same file the franka-rs test loads (tests/fixtures/fr3_robot.urdf).
//
// Each line is: <tag> <input values...> <output values...>, full-precision decimals, in the order
// the companion test (source/model/tests.rs, model_matches_libfranka) expects. Frames are encoded as an index 0..9 matching
// franka::Frame (kJoint1..kJoint7, kFlange, kEndEffector, kStiffness). All 4x4 transforms and the
// Jacobian/mass matrices are column-major, matching franka-rs's nalgebra column-major storage.

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <random>

#include <franka/model.h>

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

using franka::Frame;
using franka::Model;

// Prints one case: the tag, then the inputs, then libfranka's outputs.
static void emit(const std::string& tag,
                 const std::vector<double>& inputs,
                 const std::vector<double>& outputs) {
  std::cout << tag;
  for (double value : inputs) {
    std::cout << ' ' << value;
  }
  for (double value : outputs) {
    std::cout << ' ' << value;
  }
  std::cout << '\n';
}

// Appends every element of an array to a vector.
template <std::size_t N>
static void append(std::vector<double>& values, const std::array<double, N>& array) {
  values.insert(values.end(), array.begin(), array.end());
}

// The identity 4x4 transform in column-major order.
static const std::array<double, 16> IDENTITY = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

// The ten model frames, paired with the index the companion test maps back to a franka-rs Frame.
static const std::array<std::pair<Frame, int>, 10> FRAMES = {{{Frame::kJoint1, 0},
                                                              {Frame::kJoint2, 1},
                                                              {Frame::kJoint3, 2},
                                                              {Frame::kJoint4, 3},
                                                              {Frame::kJoint5, 4},
                                                              {Frame::kJoint6, 5},
                                                              {Frame::kJoint7, 6},
                                                              {Frame::kFlange, 7},
                                                              {Frame::kEndEffector, 8},
                                                              {Frame::kStiffness, 9}}};

// Emits pose and both Jacobians for every frame at one configuration and frame-offset pair.
static void emit_kinematics(const Model& model,
                            const std::array<double, 7>& q,
                            const std::array<double, 16>& f_t_ee,
                            const std::array<double, 16>& ee_t_k) {
  for (const auto& [frame, index] : FRAMES) {
    std::vector<double> inputs;
    append(inputs, q);
    append(inputs, f_t_ee);
    append(inputs, ee_t_k);
    inputs.push_back(index);

    auto pose = model.pose(frame, q, f_t_ee, ee_t_k);
    emit("pose", inputs, std::vector<double>(pose.begin(), pose.end()));
    auto zero = model.zeroJacobian(frame, q, f_t_ee, ee_t_k);
    emit("zero_jacobian", inputs, std::vector<double>(zero.begin(), zero.end()));
    auto body = model.bodyJacobian(frame, q, f_t_ee, ee_t_k);
    emit("body_jacobian", inputs, std::vector<double>(body.begin(), body.end()));
  }
}

// Emits pinocchio's Coriolis matrix C(q, dq) (computeCoriolisMatrix, the matrix libfranka's
// deprecated coriolis overload multiplies by dq), with the payload attached to the last joint
// exactly as libfranka's RobotModel::updateInertiaIfNeeded does. libfranka does not expose the
// matrix, so pinocchio is called directly on the same URDF.
static void emit_coriolis_matrix(const std::string& urdf,
                                 const std::array<double, 7>& q,
                                 const std::array<double, 7>& dq,
                                 const std::array<double, 9>& inertia,
                                 double mass,
                                 const std::array<double, 3>& com) {
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf, model);
  const pinocchio::FrameIndex last_link = model.getFrameId("link8");
  const pinocchio::JointIndex last_joint = model.frames[last_link].parentJoint;
  const pinocchio::Inertia payload(mass, Eigen::Map<const Eigen::Vector3d>(com.data()),
                                   Eigen::Map<const Eigen::Matrix3d>(inertia.data(), 3, 3));
  model.inertias[last_joint] = model.inertias[last_joint] + model.frames[last_link].placement.act(payload);
  pinocchio::Data data(model);
  const Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), 7);
  const Eigen::VectorXd dq_eigen = Eigen::Map<const Eigen::VectorXd>(dq.data(), 7);
  pinocchio::computeCoriolisMatrix(model, data, q_eigen, dq_eigen);
  std::vector<double> inputs;
  append(inputs, q);
  append(inputs, dq);
  append(inputs, inertia);
  inputs.push_back(mass);
  append(inputs, com);
  const Eigen::MatrixXd& c = data.C;  // column-major
  emit("coriolis_matrix", inputs, std::vector<double>(c.data(), c.data() + 49));
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: model <path-to-urdf>\n";
    return 1;
  }
  std::ifstream file(argv[1]);
  if (!file) {
    std::cerr << "cannot open URDF: " << argv[1] << '\n';
    return 1;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string urdf = buffer.str();
  Model model(urdf);

  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

  // Lets the test confirm it loaded the same URDF this data was generated from.
  std::cout << "urdf_bytes " << urdf.size() << '\n';

  const std::array<double, 7> q_zero = {0, 0, 0, 0, 0, 0, 0};
  const std::array<double, 7> q_ready = {
      0, 0, 0, -0.75 * M_PI, 0, 0.75 * M_PI, 0};
  const std::array<double, 7> q_moved = {0.001, 0.001, 0.001, -2.3552, 0.001, 2.3572, 0.001};

  // Kinematics with identity end-effector and stiffness offsets, across all frames.
  emit_kinematics(model, q_zero, IDENTITY, IDENTITY);
  emit_kinematics(model, q_ready, IDENTITY, IDENTITY);
  emit_kinematics(model, q_moved, IDENTITY, IDENTITY);

  // Kinematics with non-identity offsets, to exercise the end-effector/stiffness frame composition.
  const std::array<double, 16> f_t_ee = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0.1034, 1};
  const std::array<double, 16> ee_t_k = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0.05, 0, 0, 1};
  emit_kinematics(model, q_moved, f_t_ee, ee_t_k);

  // Loads: no load, and the Franka Hand (mass, centre of mass, inertia about the centre of mass).
  struct Load {
    std::array<double, 9> inertia;
    double mass;
    std::array<double, 3> com;
  };
  const Load no_load = {{0, 0, 0, 0, 0, 0, 0, 0, 0}, 0.0, {0, 0, 0}};
  const Load hand = {{0.001, 0, 0, 0, 0.0025, 0, 0, 0, 0.0017}, 0.73, {0.01, 0.0, 0.03}};
  // A symmetric load inertia with off-diagonal terms, to exercise the full inertia path.
  const Load hand_offdiag = {
      {0.0012, 0.0002, 0.0001, 0.0002, 0.0026, 0.0003, 0.0001, 0.0003, 0.0018},
      0.73,
      {0.01, 0.0, 0.03}};

  const std::array<double, 3> gravity_earth = {0.0, 0.0, -9.81};

  // Mass matrix and gravity torques for several configurations and loads.
  for (const auto& q : {q_zero, q_ready, q_moved}) {
    for (const Load& load : {no_load, hand, hand_offdiag}) {
      std::vector<double> mass_inputs;
      append(mass_inputs, q);
      append(mass_inputs, load.inertia);
      mass_inputs.push_back(load.mass);
      append(mass_inputs, load.com);
      auto mass = model.mass(q, load.inertia, load.mass, load.com);
      emit("mass", mass_inputs, std::vector<double>(mass.begin(), mass.end()));

      std::vector<double> gravity_inputs;
      append(gravity_inputs, q);
      gravity_inputs.push_back(load.mass);
      append(gravity_inputs, load.com);
      append(gravity_inputs, gravity_earth);
      auto gravity = model.gravity(q, load.mass, load.com, gravity_earth);
      emit("gravity", gravity_inputs, std::vector<double>(gravity.begin(), gravity.end()));
    }
  }

  // Coriolis torques for several configuration/velocity/load combinations.
  const std::array<double, 7> dq_unit = {1, 1, 1, 1, 1, 1, 1};
  const std::array<double, 7> dq_mixed = {0.3, -0.2, 0.5, 0.1, -0.4, 0.2, 0.6};
  struct Coriolis {
    std::array<double, 7> q, dq;
    Load load;
  };
  for (const Coriolis& c : {Coriolis{q_zero, dq_unit, hand},
                            Coriolis{q_ready, dq_mixed, no_load},
                            Coriolis{q_moved, dq_mixed, hand},
                            Coriolis{q_moved, dq_mixed, hand_offdiag}}) {
    std::vector<double> inputs;
    append(inputs, c.q);
    append(inputs, c.dq);
    append(inputs, c.load.inertia);
    inputs.push_back(c.load.mass);
    append(inputs, c.load.com);
    append(inputs, gravity_earth);
    auto coriolis = model.coriolis(c.q, c.dq, c.load.inertia, c.load.mass, c.load.com, gravity_earth);
    emit("coriolis", inputs, std::vector<double>(coriolis.begin(), coriolis.end()));
  }

  // Coriolis matrices: the Coriolis cases above, then seeded random configurations, velocities and
  // payloads (mass 0-2 kg, center of mass within 0.1 m, diagonal-dominant symmetric inertia).
  for (const Coriolis& c : {Coriolis{q_zero, dq_unit, hand},
                            Coriolis{q_ready, dq_mixed, no_load},
                            Coriolis{q_moved, dq_mixed, hand},
                            Coriolis{q_moved, dq_mixed, hand_offdiag}}) {
    emit_coriolis_matrix(urdf, c.q, c.dq, c.load.inertia, c.load.mass, c.load.com);
  }
  // std::uniform_real_distribution is implementation-defined, so regenerating with another C++
  // standard library can produce different random cases; the committed data file is the reference.
  std::mt19937_64 rng(20261009);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for (int index = 0; index < 20; index++) {
    std::array<double, 7> q{}, dq{};
    for (size_t i = 0; i < 7; i++) {
      q[i] = 2.0 * unit(rng);
      dq[i] = 2.0 * unit(rng);
    }
    const double off_diagonal = 0.0005 * unit(rng);
    const std::array<double, 9> inertia = {0.002 + 0.001 * unit(rng), off_diagonal, 0.0,
                                           off_diagonal, 0.003 + 0.001 * unit(rng), 0.0,
                                           0.0, 0.0, 0.002 + 0.001 * unit(rng)};
    const double mass = 1.0 + unit(rng);
    const std::array<double, 3> com = {0.1 * unit(rng), 0.1 * unit(rng), 0.1 * unit(rng)};
    emit_coriolis_matrix(urdf, q, dq, inertia, mass, com);
  }

  return 0;
}
