// Runs libfranka's low-pass filter and rate-limiting functions on chosen valid inputs and prints
// each case's inputs and libfranka's outputs, for comparison against the franka-rs ports in
// source/lowpass_filter.rs and source/rate_limiting.rs.
//
// Links the built libfranka library, so the outputs are those of the exact code franka-rs is
// compared against (no robot needed):
//
//   cmake --build validation/reference/build --target computation
//   validation/reference/build/computation > validation/data/computation_cases.txt
//
// Each line is: <tag> <input values...> <output values...>, all decimals that round-trip exactly, in
// the franka-rs argument order; franka-rs must reproduce every output bit for bit. The companion test in source/rate_limiting.rs knows each tag's input
// and output counts, re-runs the franka-rs function on the inputs, and compares against the outputs.
//
// franka-rs's lowpass_filter_joints has no libfranka counterpart (it is element-wise scalar
// filtering), so it is covered transitively by the scalar lowpass_filter cases and not emitted here.

#include <array>
#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/lowpass_filter.h>
#include <franka/rate_limiting.h>

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

// Builds a column-major 4x4 homogeneous transform from a rotation and translation.
static std::array<double, 16> transform(const Eigen::Matrix3d& rotation,
                                        const Eigen::Vector3d& translation) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.topLeftCorner<3, 3>() = rotation;
  matrix.topRightCorner<3, 1>() = translation;
  std::array<double, 16> result;  // Eigen stores column-major, matching the wire layout.
  std::copy(matrix.data(), matrix.data() + 16, result.begin());
  return result;
}

// Appends every element of an array to a vector.
template <std::size_t N>
static void append(std::vector<double>& values, const std::array<double, N>& array) {
  values.insert(values.end(), array.begin(), array.end());
}

int main() {
  std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

  const double dt = 1e-3;

  // --- scalar low-pass filter: (sample_time, current, last, cutoff) -> value ---
  for (auto [current, last, cutoff] : std::vector<std::array<double, 3>>{
           {1.0, 0.0, 100.0},
           {-2.5, 1.3, 50.0},
           {0.123456, 0.123455, 900.0}}) {
    double out = franka::lowpassFilter(dt, current, last, cutoff);
    emit("lowpass_filter", {dt, current, last, cutoff}, {out});
  }

  // --- cartesian low-pass filter: (sample_time, current[16], last[16], cutoff) -> [16] ---
  {
    struct Case {
      Eigen::Matrix3d last_rotation, current_rotation;
      Eigen::Vector3d last_translation, current_translation;
      double cutoff;
    };
    std::vector<Case> cases;
    // Small rotation delta.
    cases.push_back({Eigen::Matrix3d::Identity(),
                     Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                     {0.3, -0.2, 0.5}, {0.31, -0.19, 0.52}, 100.0});
    // Near-180-degree rotation delta (where Eigen and nalgebra interpolation can diverge).
    cases.push_back({Eigen::Matrix3d::Identity(),
                     Eigen::AngleAxisd(3.10, Eigen::Vector3d::UnitX()).toRotationMatrix(),
                     {0.0, 0.0, 0.0}, {0.1, 0.1, 0.1}, 100.0});
    // Large translation delta, moderate rotation.
    cases.push_back({Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix(),
                     Eigen::AngleAxisd(0.9, Eigen::Vector3d(1, 1, 0).normalized()).toRotationMatrix(),
                     {-0.5, 0.5, 1.0}, {0.5, -0.5, 0.2}, 30.0});
    for (const Case& c : cases) {
      auto last = transform(c.last_rotation, c.last_translation);
      auto current = transform(c.current_rotation, c.current_translation);
      auto out = franka::cartesianLowpassFilter(dt, current, last, c.cutoff);
      std::vector<double> inputs{dt};
      append(inputs, current);
      append(inputs, last);
      inputs.push_back(c.cutoff);
      emit("cartesian_lowpass_filter", inputs, std::vector<double>(out.begin(), out.end()));
    }
  }

  // --- limit_rate_torques: (max_derivatives[7], commanded[7], last_commanded[7]) -> [7] ---
  {
    std::array<double, 7> max_rate{900, 900, 900, 900, 900, 900, 900};
    std::array<double, 7> last{1, -2, 3, -4, 5, -6, 7};
    // Mix of within-rate and exceeding-rate commands.
    std::array<double, 7> commanded{1.1, -2.0, 5.0, -3.9, 10.0, -6.05, 6.0};
    auto out = franka::limitRate(max_rate, commanded, last);
    std::vector<double> inputs;
    append(inputs, max_rate);
    append(inputs, commanded);
    append(inputs, last);
    emit("limit_rate_torques", inputs, std::vector<double>(out.begin(), out.end()));
  }

  // --- limit_rate_velocity (scalar): (upper, lower, max_acc, max_jerk, cmd_vel, last_vel, last_acc)
  for (auto [cmd, last_vel, last_acc] : std::vector<std::array<double, 3>>{
           {1.0, 0.5, 0.0}, {2.0, 1.9, 10.0}, {-3.0, 0.0, -5.0}}) {
    double out = franka::limitRate(2.0, -2.0, 10.0, 5000.0, cmd, last_vel, last_acc);
    emit("limit_rate_velocity", {2.0, -2.0, 10.0, 5000.0, cmd, last_vel, last_acc}, {out});
  }

  // --- limit_rate_position (scalar): (upper, lower, max_acc, max_jerk, cmd_pos, last_pos, last_vel,
  //     last_acc) ---
  for (auto [cmd, last_pos, last_vel, last_acc] : std::vector<std::array<double, 4>>{
           {0.11, 0.1, 0.5, 0.0}, {0.5, 0.1, 2.0, 10.0}, {-0.2, 0.0, -1.0, -5.0}}) {
    double out = franka::limitRate(2.0, -2.0, 10.0, 5000.0, cmd, last_pos, last_vel, last_acc);
    emit("limit_rate_position", {2.0, -2.0, 10.0, 5000.0, cmd, last_pos, last_vel, last_acc}, {out});
  }

  // --- limit_rate_joint_velocities: 7-arrays (upper, lower, max_acc, max_jerk, cmd, last_vel,
  //     last_acc) -> [7] ---
  {
    std::array<double, 7> upper{2, 2, 2, 2, 2, 2, 2};
    std::array<double, 7> lower{-2, -2, -2, -2, -2, -2, -2};
    std::array<double, 7> max_acc{10, 10, 10, 10, 10, 10, 10};
    std::array<double, 7> max_jerk{5000, 5000, 5000, 5000, 5000, 5000, 5000};
    std::array<double, 7> cmd{0.5, 1.0, -0.5, 2.0, -2.0, 0.1, 1.5};
    std::array<double, 7> last_vel{0.4, 0.9, -0.4, 1.9, -1.8, 0.0, 1.4};
    std::array<double, 7> last_acc{0, 5, -5, 10, -10, 0, 2};
    auto out = franka::limitRate(upper, lower, max_acc, max_jerk, cmd, last_vel, last_acc);
    std::vector<double> inputs;
    for (const auto* a : {&upper, &lower, &max_acc, &max_jerk, &cmd, &last_vel, &last_acc}) {
      append(inputs, *a);
    }
    emit("limit_rate_joint_velocities", inputs, std::vector<double>(out.begin(), out.end()));
  }

  // --- limit_rate_joint_positions: 7-arrays (upper, lower, max_acc, max_jerk, cmd, last_pos,
  //     last_vel, last_acc) -> [7] ---
  {
    std::array<double, 7> upper{2, 2, 2, 2, 2, 2, 2};
    std::array<double, 7> lower{-2, -2, -2, -2, -2, -2, -2};
    std::array<double, 7> max_acc{10, 10, 10, 10, 10, 10, 10};
    std::array<double, 7> max_jerk{5000, 5000, 5000, 5000, 5000, 5000, 5000};
    std::array<double, 7> cmd{0.11, 0.3, -0.2, 0.5, -0.5, 0.05, 0.4};
    std::array<double, 7> last_pos{0.1, 0.1, -0.1, 0.1, -0.1, 0.0, 0.3};
    std::array<double, 7> last_vel{0.4, 0.9, -0.4, 1.9, -1.8, 0.0, 1.4};
    std::array<double, 7> last_acc{0, 5, -5, 10, -10, 0, 2};
    auto out = franka::limitRate(upper, lower, max_acc, max_jerk, cmd, last_pos, last_vel, last_acc);
    std::vector<double> inputs;
    for (const auto* a : {&upper, &lower, &max_acc, &max_jerk, &cmd, &last_pos, &last_vel, &last_acc}) {
      append(inputs, *a);
    }
    emit("limit_rate_joint_positions", inputs, std::vector<double>(out.begin(), out.end()));
  }

  // --- limit_rate_cartesian_velocity: (6 scalar limits, cmd[6], last_vel[6], last_acc[6]) -> [6] ---
  {
    std::array<double, 6> cmd{0.5, -0.3, 0.8, 0.2, -0.1, 0.4};
    std::array<double, 6> last_vel{0.4, -0.2, 0.7, 0.1, 0.0, 0.3};
    std::array<double, 6> last_acc{1.0, -1.0, 2.0, 0.5, -0.5, 1.0};
    auto out = franka::limitRate(1.7, 13.0, 6500.0, 2.5, 25.0, 12500.0, cmd, last_vel, last_acc);
    std::vector<double> inputs{1.7, 13.0, 6500.0, 2.5, 25.0, 12500.0};
    append(inputs, cmd);
    append(inputs, last_vel);
    append(inputs, last_acc);
    emit("limit_rate_cartesian_velocity", inputs, std::vector<double>(out.begin(), out.end()));
  }

  // --- limit_rate_cartesian_pose: (6 scalar limits, cmd[16], last[16], last_twist[6], last_acc[6])
  //     -> [16] ---
  {
    struct Case {
      Eigen::Matrix3d last_rotation, commanded_rotation;
      Eigen::Vector3d last_translation, commanded_translation;
    };
    std::vector<Case> cases;
    // Small delta.
    cases.push_back({Eigen::Matrix3d::Identity(),
                     Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                     {0.4, 0.0, 0.3}, {0.405, 0.001, 0.3}});
    // Near-180-degree rotation delta (axis-angle extraction edge).
    cases.push_back({Eigen::Matrix3d::Identity(),
                     Eigen::AngleAxisd(3.10, Eigen::Vector3d::UnitY()).toRotationMatrix(),
                     {0.4, 0.0, 0.3}, {0.4, 0.0, 0.3}});
    for (const Case& c : cases) {
      auto last = transform(c.last_rotation, c.last_translation);
      auto commanded = transform(c.commanded_rotation, c.commanded_translation);
      std::array<double, 6> last_twist{0, 0, 0, 0, 0, 0};
      std::array<double, 6> last_acc{0, 0, 0, 0, 0, 0};
      auto out =
          franka::limitRate(1.7, 13.0, 6500.0, 2.5, 25.0, 12500.0, commanded, last, last_twist, last_acc);
      std::vector<double> inputs{1.7, 13.0, 6500.0, 2.5, 25.0, 12500.0};
      append(inputs, commanded);
      append(inputs, last);
      append(inputs, last_twist);
      append(inputs, last_acc);
      emit("limit_rate_cartesian_pose", inputs, std::vector<double>(out.begin(), out.end()));
    }
  }

  // --- Randomized cases for the rotation arithmetic (seeded, so the file is reproducible) ---
  // Rotations are rounded to single precision, as the robot reports them, so they are close to but
  // not exactly orthonormal; deltas cover unchanged, small, large and near-180-degree rotations.
  {
    std::mt19937_64 rng(20261009);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    auto random_axis = [&]() { return Eigen::Vector3d(unit(rng), unit(rng), unit(rng)).normalized(); };
    auto as_float = [](const Eigen::Matrix3d& m) {
      Eigen::Matrix3d r;
      for (int i = 0; i < 9; i++) r.data()[i] = static_cast<double>(static_cast<float>(m.data()[i]));
      return r;
    };
    const std::array<double, 6> deltas{{0.0, 1e-9, 1e-4, 0.02, 1.0, 3.1}};
    for (int index = 0; index < 240; index++) {
      double delta = deltas[index % deltas.size()];
      Eigen::Matrix3d last_rotation = Eigen::AngleAxisd(3.0 * unit(rng), random_axis()).toRotationMatrix();
      Eigen::Matrix3d current_rotation = Eigen::AngleAxisd(delta, random_axis()).toRotationMatrix() * last_rotation;
      if (index % 2 == 0) {
        last_rotation = as_float(last_rotation);
        current_rotation = as_float(current_rotation);
      }
      Eigen::Vector3d last_translation(0.5 * unit(rng), 0.5 * unit(rng), 0.5 + 0.3 * unit(rng));
      Eigen::Vector3d current_translation = last_translation + Eigen::Vector3d(unit(rng), unit(rng), unit(rng)) * 0.002;
      auto last = transform(last_rotation, last_translation);
      auto current = transform(current_rotation, current_translation);

      double cutoff = index % 3 == 0 ? 100.0 : (index % 3 == 1 ? 10.0 : 900.0);
      auto filtered = franka::cartesianLowpassFilter(dt, current, last, cutoff);
      std::vector<double> filter_inputs{dt};
      append(filter_inputs, current);
      append(filter_inputs, last);
      filter_inputs.push_back(cutoff);
      emit("cartesian_lowpass_filter", filter_inputs, std::vector<double>(filtered.begin(), filtered.end()));

      std::array<double, 6> last_twist{}, last_acc{};
      for (size_t i = 0; i < 6; i++) {
        last_twist[i] = 0.5 * unit(rng);
        last_acc[i] = 4.0 * unit(rng);
      }
      auto limited = franka::limitRate(franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                                       franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                                       franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk, current,
                                       last, last_twist, last_acc);
      std::vector<double> limit_inputs{franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                                       franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                                       franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk};
      append(limit_inputs, current);
      append(limit_inputs, last);
      append(limit_inputs, last_twist);
      append(limit_inputs, last_acc);
      emit("limit_rate_cartesian_pose", limit_inputs, std::vector<double>(limited.begin(), limited.end()));
    }

    // Cartesian velocities, including last velocities above the maximum.
    for (int index = 0; index < 200; index++) {
      std::array<double, 6> commanded{}, last{}, last_acc{};
      double reach = index % 4 == 0 ? 4.0 : 1.0;
      for (size_t i = 0; i < 6; i++) {
        last[i] = reach * unit(rng);
        commanded[i] = last[i] + 0.01 * unit(rng);
        last_acc[i] = 5.0 * unit(rng);
      }
      auto out = franka::limitRate(franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                                   franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                                   franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk, commanded, last,
                                   last_acc);
      std::vector<double> inputs{franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                                 franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                                 franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk};
      append(inputs, commanded);
      append(inputs, last);
      append(inputs, last_acc);
      emit("limit_rate_cartesian_velocity", inputs, std::vector<double>(out.begin(), out.end()));
    }
  }

  // --- Edge cases for the rotation arithmetic: exact rotations with zero entries (axis
  // permutations, 90/180-degree turns), a reflection, a column scaled within the homogeneity
  // tolerance, an unchanged rotation (pure translation), and a rotation changed by one ulp (the
  // angle-axis stableNorm path). Each pair is run through both the filter and the pose limiter.
  {
    std::vector<Eigen::Matrix3d> rotations;
    rotations.push_back(Eigen::Matrix3d::Identity());
    Eigen::Matrix3d permutation;
    permutation << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    rotations.push_back(permutation);
    rotations.push_back(permutation.transpose());
    for (int axis = 0; axis < 3; axis++) {
      for (double angle : {M_PI / 2, M_PI, -M_PI / 2}) {
        Eigen::Matrix3d r = Eigen::AngleAxisd(angle, Eigen::Vector3d::Unit(axis)).toRotationMatrix();
        for (int i = 0; i < 9; i++) r.data()[i] = std::round(r.data()[i]);  // exact zeros and ones
        rotations.push_back(r);
      }
    }
    Eigen::Matrix3d reflection = Eigen::Vector3d(1.0, 1.0, -1.0).asDiagonal();
    rotations.push_back(reflection);
    Eigen::Matrix3d scaled = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    scaled.col(1) *= 1.0 + 9e-6;
    rotations.push_back(scaled);

    auto run_pair = [&](const Eigen::Matrix3d& last_rotation, const Eigen::Matrix3d& current_rotation) {
      auto last = transform(last_rotation, {0.3, -0.1, 0.5});
      auto current = transform(current_rotation, {0.301, -0.1, 0.5});
      auto filtered = franka::cartesianLowpassFilter(dt, current, last, 100.0);
      std::vector<double> filter_inputs{dt};
      append(filter_inputs, current);
      append(filter_inputs, last);
      filter_inputs.push_back(100.0);
      emit("cartesian_lowpass_filter", filter_inputs, std::vector<double>(filtered.begin(), filtered.end()));
      std::array<double, 6> zero{};
      auto limited = franka::limitRate(franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                                       franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                                       franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk, current,
                                       last, zero, zero);
      std::vector<double> limit_inputs{franka::kMaxTranslationalVelocity, franka::kMaxTranslationalAcceleration,
                                       franka::kMaxTranslationalJerk, franka::kMaxRotationalVelocity,
                                       franka::kMaxRotationalAcceleration, franka::kMaxRotationalJerk};
      append(limit_inputs, current);
      append(limit_inputs, last);
      append(limit_inputs, zero);
      append(limit_inputs, zero);
      emit("limit_rate_cartesian_pose", limit_inputs, std::vector<double>(limited.begin(), limited.end()));
    };
    for (const Eigen::Matrix3d& rotation : rotations) {
      run_pair(rotation, rotation);  // unchanged rotation
      Eigen::Matrix3d nudged = rotation;
      for (int i = 0; i < 9; i++) {
        if (nudged.data()[i] != 0.0) {
          nudged.data()[i] = std::nextafter(nudged.data()[i], 2.0);  // one ulp
          break;
        }
      }
      run_pair(rotation, nudged);
      for (const Eigen::Matrix3d& other : rotations) {
        run_pair(rotation, other);
      }
    }
  }

  return 0;
}
