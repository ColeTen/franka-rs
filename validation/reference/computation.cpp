// Runs libfranka's low-pass filter and rate-limiting functions on chosen valid inputs and prints
// each case's inputs and libfranka's outputs, for comparison against the franka-rs ports in
// source/lowpass_filter.rs and source/rate_limiting.rs.
//
// Needs only Eigen and libfranka's headers plus the two source files it exercises (no Pinocchio, no
// built library, no robot), so it compiles natively, e.g.:
//
//   c++ -std=c++17 -I external/libfranka/include -I /opt/homebrew/include/eigen3 \
//       validation/reference/computation.cpp \
//       external/libfranka/src/lowpass_filter.cpp external/libfranka/src/rate_limiting.cpp \
//       -o computation
//   ./computation > validation/data/computation_cases.txt
//
// Each line is: <tag> <input values...> <output values...>, all full-precision decimals, in the
// franka-rs argument order. The companion test in source/rate_limiting.rs knows each tag's input
// and output counts, re-runs the franka-rs function on the inputs, and compares against the outputs.
//
// franka-rs's lowpass_filter_joints has no libfranka counterpart (it is element-wise scalar
// filtering), so it is covered transitively by the scalar lowpass_filter cases and not emitted here.

#include <array>
#include <iomanip>
#include <iostream>
#include <limits>
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

  return 0;
}
