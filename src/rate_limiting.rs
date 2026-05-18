#![allow(clippy::too_many_arguments)]

use nalgebra::Vector3;

use crate::constants::DELTA_T;

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

/// Limit the rate of per-joint torque commands.
///
/// Clamps the derivative of each joint value to `max_derivatives[i]`.
pub fn limit_rate_torques(
    max_derivatives: &[f64; 7],
    commanded: &[f64; 7],
    last_commanded: &[f64; 7],
) -> [f64; 7] {
    let mut limited = [0.0; 7];
    for i in 0..7 {
        let derivative = (commanded[i] - last_commanded[i]) / DELTA_T;
        let clamped = derivative.clamp(-max_derivatives[i], max_derivatives[i]);
        limited[i] = last_commanded[i] + clamped * DELTA_T;
    }
    limited
}

/// Limit the rate of a single joint velocity value.
pub fn limit_rate_velocity(
    upper_limit: f64,
    lower_limit: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_velocity: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> f64 {
    // Differentiate to get jerk
    let commanded_jerk =
        (((commanded_velocity - last_commanded_velocity) / DELTA_T) - last_commanded_acceleration)
            / DELTA_T;

    // Limit jerk and integrate to get acceleration
    let commanded_acceleration =
        last_commanded_acceleration + commanded_jerk.clamp(-max_jerk, max_jerk) * DELTA_T;

    // Compute safe acceleration limits based on velocity bounds
    let safe_max_acceleration = ((max_jerk / max_acceleration)
        * (upper_limit - last_commanded_velocity))
        .min(max_acceleration);
    let safe_min_acceleration = ((max_jerk / max_acceleration)
        * (lower_limit - last_commanded_velocity))
        .max(-max_acceleration);

    // Limit acceleration and integrate to get velocity
    last_commanded_velocity
        + commanded_acceleration.clamp(safe_min_acceleration, safe_max_acceleration) * DELTA_T
}

/// Limit the rate of a single joint position value.
pub fn limit_rate_position(
    upper_velocity_limit: f64,
    lower_velocity_limit: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_position: f64,
    last_commanded_position: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> f64 {
    // Convert position command to velocity command, then limit the velocity
    let commanded_velocity = (commanded_position - last_commanded_position) / DELTA_T;
    let limited_velocity = limit_rate_velocity(
        upper_velocity_limit,
        lower_velocity_limit,
        max_acceleration,
        max_jerk,
        commanded_velocity,
        last_commanded_velocity,
        last_commanded_acceleration,
    );
    last_commanded_position + limited_velocity * DELTA_T
}

/// Limit the rate of joint velocities (all 7 joints).
pub fn limit_rate_joint_velocities(
    upper_limits: &[f64; 7],
    lower_limits: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded: &[f64; 7],
    last_commanded: &[f64; 7],
    last_acceleration: &[f64; 7],
) -> [f64; 7] {
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
        );
    }
    limited
}

/// Limit the rate of joint positions (all 7 joints).
pub fn limit_rate_joint_positions(
    upper_velocity_limits: &[f64; 7],
    lower_velocity_limits: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded: &[f64; 7],
    last_commanded: &[f64; 7],
    last_velocity: &[f64; 7],
    last_acceleration: &[f64; 7],
) -> [f64; 7] {
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
        );
    }
    limited
}

/// Limit the rate of a 3D vector (translational or rotational velocity).
fn limit_rate_vector3(
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded: &Vector3<f64>,
    last_commanded: &Vector3<f64>,
    last_acceleration: &Vector3<f64>,
) -> Vector3<f64> {
    // Differentiate to get jerk
    let commanded_jerk =
        ((commanded - last_commanded) / DELTA_T - last_acceleration) / DELTA_T;

    // Limit jerk and integrate to get desired acceleration
    let mut commanded_acceleration = *last_acceleration;
    let jerk_norm = commanded_jerk.norm();
    if jerk_norm > NORM_EPS {
        commanded_acceleration +=
            (commanded_jerk / jerk_norm) * commanded_jerk.norm().clamp(-max_jerk, max_jerk) * DELTA_T;
    }

    // Compute distance to max velocity along the acceleration direction
    let accel_norm = commanded_acceleration.norm();
    if accel_norm <= NORM_EPS {
        return *last_commanded;
    }

    let unit_accel = commanded_acceleration / accel_norm;
    let dot_product = unit_accel.dot(last_commanded);
    let discriminant =
        dot_product * dot_product - last_commanded.norm_squared() + max_velocity * max_velocity;
    let distance_to_max = -dot_product + discriminant.max(0.0).sqrt();

    // Compute safe acceleration limit
    let safe_max_acceleration =
        ((max_jerk / max_acceleration) * distance_to_max).min(max_acceleration);

    // Limit acceleration and integrate to get velocity
    let mut limited = *last_commanded;
    if accel_norm > NORM_EPS {
        limited += unit_accel * accel_norm.min(safe_max_acceleration) * DELTA_T;
    }

    limited
}

/// Limit the rate of a Cartesian velocity (6D twist: [vx, vy, vz, wx, wy, wz]).
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
) -> [f64; 6] {
    let cmd_trans = Vector3::new(commanded[0], commanded[1], commanded[2]);
    let cmd_rot = Vector3::new(commanded[3], commanded[4], commanded[5]);
    let last_trans = Vector3::new(last_commanded[0], last_commanded[1], last_commanded[2]);
    let last_rot = Vector3::new(last_commanded[3], last_commanded[4], last_commanded[5]);
    let last_accel_trans =
        Vector3::new(last_acceleration[0], last_acceleration[1], last_acceleration[2]);
    let last_accel_rot =
        Vector3::new(last_acceleration[3], last_acceleration[4], last_acceleration[5]);

    let limited_trans = limit_rate_vector3(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        &cmd_trans,
        &last_trans,
        &last_accel_trans,
    );

    let limited_rot = limit_rate_vector3(
        max_rotational_velocity,
        max_rotational_acceleration,
        max_rotational_jerk,
        &cmd_rot,
        &last_rot,
        &last_accel_rot,
    );

    [
        limited_trans.x,
        limited_trans.y,
        limited_trans.z,
        limited_rot.x,
        limited_rot.y,
        limited_rot.z,
    ]
}

/// Limit the rate of a Cartesian pose (4x4 column-major homogeneous transform).
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
) -> [f64; 16] {
    use nalgebra::{Matrix3, Matrix4, Rotation3};

    let cmd_mat = Matrix4::from_column_slice(commanded);
    let last_mat = Matrix4::from_column_slice(last_commanded);

    let cmd_translation = Vector3::new(cmd_mat[(0, 3)], cmd_mat[(1, 3)], cmd_mat[(2, 3)]);
    let last_translation = Vector3::new(last_mat[(0, 3)], last_mat[(1, 3)], last_mat[(2, 3)]);

    // Compute translational velocity from pose difference
    let trans_vel = (cmd_translation - last_translation) / DELTA_T;

    // Compute rotational velocity from rotation difference
    let cmd_rot: Matrix3<f64> = cmd_mat.fixed_view::<3, 3>(0, 0).into_owned();
    let last_rot: Matrix3<f64> = last_mat.fixed_view::<3, 3>(0, 0).into_owned();

    let rot_diff = Rotation3::from_matrix_unchecked(cmd_rot * last_rot.transpose());
    let angle_axis = rot_diff.scaled_axis();
    let rot_vel = angle_axis / DELTA_T;

    // Build the twist and limit it
    let twist = [
        trans_vel.x,
        trans_vel.y,
        trans_vel.z,
        rot_vel.x,
        rot_vel.y,
        rot_vel.z,
    ];

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
    );

    // Integrate limited twist to get limited pose
    let limited_translation = last_translation
        + Vector3::new(limited_twist[0], limited_twist[1], limited_twist[2]) * DELTA_T;

    let omega = Vector3::new(limited_twist[3], limited_twist[4], limited_twist[5]);
    let omega_norm = omega.norm();

    let limited_rot = if omega_norm > NORM_EPS {
        let w_norm = omega / omega_norm;
        let theta = DELTA_T * omega_norm;
        // Rodrigues' rotation formula
        let omega_skew = Matrix3::new(
            0.0, -w_norm.z, w_norm.y, w_norm.z, 0.0, -w_norm.x, -w_norm.y, w_norm.x, 0.0,
        );
        let rotation =
            Matrix3::identity() + theta.sin() * omega_skew + (1.0 - theta.cos()) * (omega_skew * omega_skew);
        rotation * last_rot
    } else {
        last_rot
    };

    let mut result_mat = Matrix4::identity();
    result_mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&limited_rot);
    result_mat[(0, 3)] = limited_translation.x;
    result_mat[(1, 3)] = limited_translation.y;
    result_mat[(2, 3)] = limited_translation.z;

    let mut result = [0.0; 16];
    result.copy_from_slice(result_mat.as_slice());
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limit_rate_torques_no_change() {
        let values = [1.0; 7];
        let result = limit_rate_torques(&MAX_TORQUE_RATE, &values, &values);
        for i in 0..7 {
            assert!((result[i] - values[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn limit_rate_torques_clamps_large_derivative() {
        let last = [0.0; 7];
        // A change of 10 in 1ms = 10000 Nm/s, which exceeds max of ~999 Nm/s
        let commanded = [10.0; 7];
        let result = limit_rate_torques(&MAX_TORQUE_RATE, &commanded, &last);
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
        let result = limit_rate_torques(&MAX_TORQUE_RATE, &commanded, &last);
        for i in 0..7 {
            assert!((result[i] - commanded[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn limit_rate_velocity_no_change() {
        let result = limit_rate_velocity(2.62, -2.62, 10.0, 5000.0, 0.5, 0.5, 0.0);
        assert!((result - 0.5).abs() < 1e-10);
    }

    #[test]
    fn limit_rate_position_no_change() {
        let result = limit_rate_position(2.62, -2.62, 10.0, 5000.0, 1.0, 1.0, 0.0, 0.0);
        assert!((result - 1.0).abs() < 1e-10);
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
        );
        for v in &result {
            assert!(v.abs() < 1e-10);
        }
    }
}
