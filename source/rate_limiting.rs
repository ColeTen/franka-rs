#![allow(clippy::too_many_arguments)]

use crate::command_checks;
use crate::constants::DELTA_T;
use crate::eigen_compat;
use crate::errors::{FrankaError, FrankaResult};

/// Epsilon value for checking limits.
pub const LIMIT_EPS: f64 = 1e-3;

/// Epsilon for norm comparisons.
pub const NORM_EPS: f64 = f64::EPSILON;

/// Factor for rotational limits using the Cartesian Pose interface.
pub const FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE: f64 = 0.99;

/// Maximum torque rate per joint in Nm/s.
pub const MAX_TORQUE_RATE: [f64; 7] = [
    1000.0 - LIMIT_EPS,
    1000.0 - LIMIT_EPS,
    1000.0 - LIMIT_EPS,
    1000.0 - LIMIT_EPS,
    1000.0 - LIMIT_EPS,
    1000.0 - LIMIT_EPS,
    1000.0 - LIMIT_EPS,
];

/// Maximum joint jerk in rad/s^3.
pub const MAX_JOINT_JERK: [f64; 7] = [
    5000.0 - LIMIT_EPS,
    5000.0 - LIMIT_EPS,
    5000.0 - LIMIT_EPS,
    5000.0 - LIMIT_EPS,
    5000.0 - LIMIT_EPS,
    5000.0 - LIMIT_EPS,
    5000.0 - LIMIT_EPS,
];

/// Maximum joint acceleration in rad/s^2.
pub const MAX_JOINT_ACCELERATION: [f64; 7] = [
    10.0 - LIMIT_EPS,
    10.0 - LIMIT_EPS,
    10.0 - LIMIT_EPS,
    10.0 - LIMIT_EPS,
    10.0 - LIMIT_EPS,
    10.0 - LIMIT_EPS,
    10.0 - LIMIT_EPS,
];

/// Margin in rad/s by which joint velocity limits are tightened, to absorb numerical errors.
pub const JOINT_VELOCITY_LIMITS_TOLERANCE: [f64; 7] = [LIMIT_EPS; 7];

/// Maximum translational jerk in m/s^3.
pub const MAX_TRANSLATIONAL_JERK: f64 = 4500.0 - LIMIT_EPS;

/// Maximum translational acceleration in m/s^2.
pub const MAX_TRANSLATIONAL_ACCELERATION: f64 = 9.0 - LIMIT_EPS;

/// Maximum translational velocity in m/s.
pub const MAX_TRANSLATIONAL_VELOCITY: f64 = 3.0 - LIMIT_EPS;

/// Maximum rotational jerk in rad/s^3.
pub const MAX_ROTATIONAL_JERK: f64 = 8500.0 - LIMIT_EPS;

/// Maximum rotational acceleration in rad/s^2.
pub const MAX_ROTATIONAL_ACCELERATION: f64 = 17.0 - LIMIT_EPS;

/// Maximum rotational velocity in rad/s.
pub const MAX_ROTATIONAL_VELOCITY: f64 = 2.5 - LIMIT_EPS;

/// Maximum elbow jerk in rad/s^3.
pub const MAX_ELBOW_JERK: f64 = 5000.0 - LIMIT_EPS;

/// Maximum elbow acceleration in rad/s^2.
pub const MAX_ELBOW_ACCELERATION: f64 = 10.0 - LIMIT_EPS;

/// Maximum elbow velocity in rad/s.
pub const MAX_ELBOW_VELOCITY: f64 = 1.5 - LIMIT_EPS;

/// Returns the smaller of `a` and `b` as C++'s `std::min(a, b)` does: `b` only if `b < a`, so a NaN
/// in `a` is kept and a NaN in `b` ignored (Rust's `f64::min` ignores either).
fn cpp_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// Returns the larger of `a` and `b` as C++'s `std::max(a, b)` does: `b` only if `a < b`.
fn cpp_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// Returns an [`FrankaError::InvalidArgument`] naming `what` unless every value is finite.
fn require_finite(values: &[f64], what: &str) -> FrankaResult<()> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(FrankaError::InvalidArgument { message: format!("{what} is infinite or NaN") })
    }
}

/// Limit the rate of per-joint torque commands.
///
/// Clamps the derivative of each joint value to `max_derivatives[i]`.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if a commanded value is not finite.
pub fn limit_rate_torques(
    max_derivatives: &[f64; 7],
    commanded: &[f64; 7],
    last_commanded: &[f64; 7],
) -> FrankaResult<[f64; 7]> {
    require_finite(commanded, "commanded value")?;
    Ok(std::array::from_fn(|i| {
        let derivative = (commanded[i] - last_commanded[i]) / DELTA_T;
        last_commanded[i] + cpp_max(cpp_min(derivative, max_derivatives[i]), -max_derivatives[i]) * DELTA_T
    }))
}

/// Limit the rate of a single joint velocity value.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if `commanded_velocity` is not finite.
pub fn limit_rate_velocity(
    upper_limit: f64,
    lower_limit: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_velocity: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> FrankaResult<f64> {
    require_finite(&[commanded_velocity], "commanded velocity")?;
    // Differentiate to get jerk
    let commanded_jerk =
        (((commanded_velocity - last_commanded_velocity) / DELTA_T) - last_commanded_acceleration)
            / DELTA_T;

    // Limit jerk and integrate to get acceleration
    let commanded_acceleration =
        last_commanded_acceleration + cpp_max(cpp_min(commanded_jerk, max_jerk), -max_jerk) * DELTA_T;

    // Compute safe acceleration limits based on velocity bounds
    let safe_max_acceleration =
        cpp_min((max_jerk / max_acceleration) * (upper_limit - last_commanded_velocity), max_acceleration);
    let safe_min_acceleration =
        cpp_max((max_jerk / max_acceleration) * (lower_limit - last_commanded_velocity), -max_acceleration);

    // Limit acceleration and integrate to get velocity. The bounds can cross when the last velocity
    // already exceeds a velocity limit; applying the upper bound first and the lower bound last
    // then yields the lower bound.
    Ok(last_commanded_velocity
        + cpp_max(cpp_min(commanded_acceleration, safe_max_acceleration), safe_min_acceleration) * DELTA_T)
}

/// Limit the rate of a single joint position value.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if `commanded_position` (or the velocity derived from it) is
/// not finite.
pub fn limit_rate_position(
    upper_velocity_limit: f64,
    lower_velocity_limit: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_position: f64,
    last_commanded_position: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> FrankaResult<f64> {
    require_finite(&[commanded_position], "commanded position")?;
    // Convert position command to velocity command, then limit the velocity
    let limited_velocity = limit_rate_velocity(
        upper_velocity_limit,
        lower_velocity_limit,
        max_acceleration,
        max_jerk,
        (commanded_position - last_commanded_position) / DELTA_T,
        last_commanded_velocity,
        last_commanded_acceleration,
    )?;
    Ok(last_commanded_position + limited_velocity * DELTA_T)
}

/// Limit the rate of joint velocities (all 7 joints).
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if a commanded velocity is not finite.
pub fn limit_rate_joint_velocities(
    upper_limits: &[f64; 7],
    lower_limits: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded: &[f64; 7],
    last_commanded: &[f64; 7],
    last_acceleration: &[f64; 7],
) -> FrankaResult<[f64; 7]> {
    require_finite(commanded, "commanded velocities")?;
    let mut limited = [0.0; 7];
    for i in 0..7 {
        limited[i] = limit_rate_velocity(
            upper_limits[i],
            lower_limits[i],
            max_acceleration[i],
            max_jerk[i],
            commanded[i],
            last_commanded[i],
            last_acceleration[i],
        )?;
    }
    Ok(limited)
}

/// Limit the rate of joint positions (all 7 joints).
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if a commanded position is not finite.
pub fn limit_rate_joint_positions(
    upper_velocity_limits: &[f64; 7],
    lower_velocity_limits: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded: &[f64; 7],
    last_commanded: &[f64; 7],
    last_velocity: &[f64; 7],
    last_acceleration: &[f64; 7],
) -> FrankaResult<[f64; 7]> {
    require_finite(commanded, "commanded positions")?;
    let mut limited = [0.0; 7];
    for i in 0..7 {
        limited[i] = limit_rate_position(
            upper_velocity_limits[i],
            lower_velocity_limits[i],
            max_acceleration[i],
            max_jerk[i],
            commanded[i],
            last_commanded[i],
            last_velocity[i],
            last_acceleration[i],
        )?;
    }
    Ok(limited)
}

/// Limit the rate of a 3D vector (translational or rotational velocity), with libfranka's
/// arithmetic (Eigen's summation order; a NaN distance to the maximum velocity leaves the
/// acceleration unlimited by it, as in libfranka).
fn limit_rate_vector3(
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded: &[f64; 3],
    last_commanded: &[f64; 3],
    last_acceleration: &[f64; 3],
) -> [f64; 3] {
    // Differentiate to get jerk
    let commanded_jerk: [f64; 3] =
        std::array::from_fn(|i| (((commanded[i] - last_commanded[i]) / DELTA_T) - last_acceleration[i]) / DELTA_T);

    // Limit jerk and integrate to get desired acceleration
    let mut commanded_acceleration = *last_acceleration;
    let jerk_norm = eigen_compat::norm3(&commanded_jerk);
    if jerk_norm > NORM_EPS {
        let limited_jerk = cpp_max(cpp_min(jerk_norm, max_jerk), -max_jerk);
        for i in 0..3 {
            commanded_acceleration[i] += ((commanded_jerk[i] / jerk_norm) * limited_jerk) * DELTA_T;
        }
    }

    // Distance to the maximum velocity along the acceleration direction, as libfranka computes it:
    // with no acceleration the direction is NaN, and when the last velocity already exceeds the
    // maximum the square root is NaN; either way cpp_min then leaves the acceleration unlimited by
    // this bound.
    let acceleration_norm = eigen_compat::norm3(&commanded_acceleration);
    let unit_acceleration = commanded_acceleration.map(|value| value / acceleration_norm);
    let dot_product = eigen_compat::dot3(&unit_acceleration, last_commanded);
    let distance_to_max = -dot_product
        + (dot_product * dot_product - eigen_compat::squared_norm3(last_commanded) + max_velocity * max_velocity).sqrt();

    // Compute safe acceleration limit
    let safe_max_acceleration = cpp_min((max_jerk / max_acceleration) * distance_to_max, max_acceleration);

    // Limit acceleration and integrate to get velocity
    let mut limited = *last_commanded;
    if acceleration_norm > NORM_EPS {
        let step = cpp_min(acceleration_norm, safe_max_acceleration);
        for i in 0..3 {
            limited[i] += (unit_acceleration[i] * step) * DELTA_T;
        }
    }
    limited
}

/// Limit the rate of a Cartesian velocity (6D twist: [vx, vy, vz, wx, wy, wz]).
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if a commanded value is not finite.
pub fn limit_rate_cartesian_velocity(
    max_translational_velocity: f64,
    max_translational_acceleration: f64,
    max_translational_jerk: f64,
    max_rotational_velocity: f64,
    max_rotational_acceleration: f64,
    max_rotational_jerk: f64,
    commanded: &[f64; 6],
    last_commanded: &[f64; 6],
    last_acceleration: &[f64; 6],
) -> FrankaResult<[f64; 6]> {
    require_finite(commanded, "O_dP_EE_c")?;
    let head = |values: &[f64; 6]| [values[0], values[1], values[2]];
    let tail = |values: &[f64; 6]| [values[3], values[4], values[5]];
    let translation = limit_rate_vector3(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        &head(commanded),
        &head(last_commanded),
        &head(last_acceleration),
    );
    let rotation = limit_rate_vector3(
        max_rotational_velocity,
        max_rotational_acceleration,
        max_rotational_jerk,
        &tail(commanded),
        &tail(last_commanded),
        &tail(last_acceleration),
    );
    Ok([translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2]])
}

/// Limit the rate of a Cartesian pose (4x4 column-major homogeneous transform), with libfranka's
/// arithmetic (rotations extracted as Eigen's `Affine3d::rotation()`, the rotation difference as
/// Eigen's `AngleAxisd`).
///
/// # Errors
/// [`FrankaError::InvalidArgument`] if `commanded` is not finite or not a homogeneous
/// transformation.
pub fn limit_rate_cartesian_pose(
    max_translational_velocity: f64,
    max_translational_acceleration: f64,
    max_translational_jerk: f64,
    max_rotational_velocity: f64,
    max_rotational_acceleration: f64,
    max_rotational_jerk: f64,
    commanded: &[f64; 16],
    last_commanded: &[f64; 16],
    last_twist: &[f64; 6],
    last_acceleration: &[f64; 6],
) -> FrankaResult<[f64; 16]> {
    require_finite(commanded, "O_T_EE_c")?;
    command_checks::check_matrix(commanded)?;
    let last_rotation = eigen_compat::affine_rotation(&eigen_compat::linear_part(last_commanded));

    // Twist from the pose difference: translational velocity, then rotational velocity.
    let mut twist = [0.0; 6];
    for i in 0..3 {
        twist[i] = (commanded[12 + i] - last_commanded[12 + i]) / DELTA_T;
    }
    let commanded_rotation = eigen_compat::affine_rotation(&eigen_compat::linear_part(commanded));
    let (angle, axis) = eigen_compat::angle_axis(&eigen_compat::mul_transpose(&commanded_rotation, &last_rotation));
    for i in 0..3 {
        twist[3 + i] = (axis[i] * angle) / DELTA_T;
    }

    let limited_twist = limit_rate_cartesian_velocity(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_velocity,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_acceleration,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_jerk,
        &twist,
        last_twist,
        last_acceleration,
    )?;

    // Integrate the limited twist: translation, then rotation (Rodrigues' formula).
    let mut result = [0.0; 16];
    result[15] = 1.0;
    for i in 0..3 {
        result[12 + i] = last_commanded[12 + i] + limited_twist[i] * DELTA_T;
    }
    let omega = [limited_twist[3], limited_twist[4], limited_twist[5]];
    let omega_norm = eigen_compat::norm3(&omega);
    let rotation = if omega_norm > NORM_EPS {
        let w = omega.map(|value| value / omega_norm);
        let theta = DELTA_T * omega_norm;
        let skew = [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]];
        let (sin_theta, one_minus_cos) = (theta.sin(), 1.0 - theta.cos());
        let skew_squared = eigen_compat::scaled_mul(one_minus_cos, &skew, &skew);
        let identity = eigen_compat::identity();
        let step: eigen_compat::Mat3 = std::array::from_fn(|i| {
            std::array::from_fn(|j| (identity[i][j] + sin_theta * skew[i][j]) + skew_squared[i][j])
        });
        eigen_compat::mul(&step, &last_rotation)
    } else {
        last_rotation
    };
    for column in 0..3 {
        for row in 0..3 {
            result[column * 4 + row] = rotation[row][column];
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limit_rate_torques_no_change() {
        let values = [1.0; 7];
        let result = limit_rate_torques(&MAX_TORQUE_RATE, &values, &values).unwrap();
        for i in 0..7 {
            assert!((result[i] - values[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn limit_rate_velocity_decelerates_when_last_velocity_exceeds_upper_limit() {
        // Last velocity 0.1 is above the upper limit 0.0, so the acceleration bounds cross
        // (safe max -50, safe min -10); the result decelerates at the lower bound.
        let result = limit_rate_velocity(0.0, -1.0, 10.0, 5000.0, 0.1, 0.1, 0.0).unwrap();
        assert!((result - (0.1 - 10.0 * DELTA_T)).abs() < 1e-12);
    }

    #[test]
    fn limit_rate_torques_clamps_large_derivative() {
        let last = [0.0; 7];
        // A change of 10 in 1ms = 10000 Nm/s, which exceeds max of ~999 Nm/s
        let commanded = [10.0; 7];
        let result = limit_rate_torques(&MAX_TORQUE_RATE, &commanded, &last).unwrap();
        for i in 0..7 {
            // Should be clamped to max_rate * dt = ~0.999
            assert!(result[i] < 1.0);
            assert!(result[i] > 0.0);
        }
    }

    #[test]
    fn limit_rate_torques_allows_small_change() {
        let last = [0.0; 7];
        // A small change well within limits
        let commanded = [0.0001; 7];
        let result = limit_rate_torques(&MAX_TORQUE_RATE, &commanded, &last).unwrap();
        for i in 0..7 {
            assert!((result[i] - commanded[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn limit_rate_velocity_no_change() {
        let result = limit_rate_velocity(2.62, -2.62, 10.0, 5000.0, 0.5, 0.5, 0.0).unwrap();
        assert!((result - 0.5).abs() < 1e-10);
    }

    #[test]
    fn limit_rate_position_no_change() {
        let result = limit_rate_position(2.62, -2.62, 10.0, 5000.0, 1.0, 1.0, 0.0, 0.0).unwrap();
        assert!((result - 1.0).abs() < 1e-10);
    }

    #[test]
    fn cartesian_velocity_above_maximum_is_not_acceleration_limited_by_the_velocity_bound() {
        // Last velocity 3.5 m/s along y exceeds the maximum, and the acceleration is along x, so
        // libfranka's distance to the maximum velocity is NaN and the acceleration (1 m/s²) passes.
        let last = [0.0, 3.5, 0.0, 0.0, 0.0, 0.0];
        let acceleration = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let commanded = [DELTA_T, 3.5, 0.0, 0.0, 0.0, 0.0];
        let result = limit_rate_cartesian_velocity(
            MAX_TRANSLATIONAL_VELOCITY,
            MAX_TRANSLATIONAL_ACCELERATION,
            MAX_TRANSLATIONAL_JERK,
            MAX_ROTATIONAL_VELOCITY,
            MAX_ROTATIONAL_ACCELERATION,
            MAX_ROTATIONAL_JERK,
            &commanded,
            &last,
            &acceleration,
        ).unwrap();
        assert!((result[0] - DELTA_T).abs() < 1e-12, "{result:?}");
        assert!((result[1] - 3.5).abs() < 1e-12, "{result:?}");
    }

    #[test]
    fn limit_rate_cartesian_velocity_zero() {
        let zero = [0.0; 6];
        let result = limit_rate_cartesian_velocity(
            MAX_TRANSLATIONAL_VELOCITY,
            MAX_TRANSLATIONAL_ACCELERATION,
            MAX_TRANSLATIONAL_JERK,
            MAX_ROTATIONAL_VELOCITY,
            MAX_ROTATIONAL_ACCELERATION,
            MAX_ROTATIONAL_JERK,
            &zero,
            &zero,
            &zero,
        ).unwrap();
        for v in &result {
            assert!(v.abs() < 1e-10);
        }
    }

    #[test]
    fn every_public_limiter_rejects_non_finite_commands() {
        let zero7 = [0.0; 7];
        let mut bad7 = zero7;
        bad7[4] = f64::INFINITY;
        let zero6 = [0.0; 6];
        let mut bad6 = zero6;
        bad6[2] = f64::NAN;
        let mut identity = [0.0; 16];
        for index in [0, 5, 10, 15] {
            identity[index] = 1.0;
        }
        let mut bad_pose = identity;
        bad_pose[13] = f64::NAN;
        let invalid = |result: FrankaResult<()>| matches!(result, Err(FrankaError::InvalidArgument { .. }));
        assert!(invalid(limit_rate_torques(&MAX_TORQUE_RATE, &bad7, &zero7).map(|_| ())));
        assert!(invalid(limit_rate_velocity(1.0, -1.0, 10.0, 5000.0, f64::NAN, 0.0, 0.0).map(|_| ())));
        assert!(invalid(limit_rate_position(1.0, -1.0, 10.0, 5000.0, f64::INFINITY, 0.0, 0.0, 0.0).map(|_| ())));
        assert!(invalid(
            limit_rate_joint_velocities(&[1.0; 7], &[-1.0; 7], &MAX_JOINT_ACCELERATION, &MAX_JOINT_JERK, &bad7, &zero7, &zero7)
                .map(|_| ())
        ));
        assert!(invalid(
            limit_rate_joint_positions(&[1.0; 7], &[-1.0; 7], &MAX_JOINT_ACCELERATION, &MAX_JOINT_JERK, &bad7, &zero7, &zero7, &zero7)
                .map(|_| ())
        ));
        assert!(invalid(limit_rate_cartesian_velocity(3.0, 9.0, 4500.0, 2.5, 17.0, 8500.0, &bad6, &zero6, &zero6).map(|_| ())));
        assert!(invalid(
            limit_rate_cartesian_pose(3.0, 9.0, 4500.0, 2.5, 17.0, 8500.0, &bad_pose, &identity, &zero6, &zero6).map(|_| ())
        ));
        let mut scaled = identity;
        scaled[0] = 2.0;
        assert!(invalid(
            limit_rate_cartesian_pose(3.0, 9.0, 4500.0, 2.5, 17.0, 8500.0, &scaled, &identity, &zero6, &zero6).map(|_| ())
        ));
    }
}
