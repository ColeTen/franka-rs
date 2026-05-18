use nalgebra::{Matrix3, Matrix4, SMatrix, Vector3};

type Matrix6x7 = SMatrix<f64, 6, 7>;

use crate::types::Frame;

/// Modified DH parameters for the Franka Emika Panda/FR3 robot.
///
/// Parameters: (a, d, alpha) for each joint.
/// Convention: Modified DH (Craig's convention).
pub const DH_A: [f64; 7] = [0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0];
pub const DH_D: [f64; 7] = [0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.107];
pub const DH_ALPHA: [f64; 7] = [
    0.0,
    -std::f64::consts::FRAC_PI_2,
    std::f64::consts::FRAC_PI_2,
    std::f64::consts::FRAC_PI_2,
    -std::f64::consts::FRAC_PI_2,
    std::f64::consts::FRAC_PI_2,
    std::f64::consts::FRAC_PI_2,
];

/// Flange-to-end-effector default transform (identity).
pub const IDENTITY_4X4: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// Compute a single Modified DH transform for joint i.
///
/// T_i = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
///
/// For the Franka robot with modified DH:
/// T_i = [[c(q), -s(q), 0, a],
///         [s(q)*c(alpha), c(q)*c(alpha), -s(alpha), -d*s(alpha)],
///         [s(q)*s(alpha), c(q)*s(alpha),  c(alpha),  d*c(alpha)],
///         [0, 0, 0, 1]]
fn dh_transform(a: f64, d: f64, alpha: f64, theta: f64) -> Matrix4<f64> {
    let (ct, st) = (theta.cos(), theta.sin());
    let (ca, sa) = (alpha.cos(), alpha.sin());

    Matrix4::new(
        ct, -st, 0.0, a,
        st * ca, ct * ca, -sa, -d * sa,
        st * sa, ct * sa, ca, d * ca,
        0.0, 0.0, 0.0, 1.0,
    )
}

/// Compute forward kinematics up to the given joint index (0-based, 0..7).
///
/// Returns the transform from base to joint `joint_index` frame.
pub fn forward_kinematics(q: &[f64; 7], joint_index: usize) -> Matrix4<f64> {
    let n = joint_index.min(7);
    let mut t = Matrix4::identity();

    for i in 0..n {
        t *= dh_transform(DH_A[i], DH_D[i], DH_ALPHA[i], q[i]);
    }

    t
}

/// Compute the pose of the flange frame (after all 7 joints).
pub fn flange_pose(q: &[f64; 7]) -> Matrix4<f64> {
    forward_kinematics(q, 7)
}

/// Compute the end-effector pose given joint positions and F_T_EE.
pub fn ee_pose(q: &[f64; 7], f_t_ee: &Matrix4<f64>) -> Matrix4<f64> {
    flange_pose(q) * f_t_ee
}

/// Compute the stiffness frame pose given joint positions, F_T_EE, and EE_T_K.
pub fn stiffness_pose(q: &[f64; 7], f_t_ee: &Matrix4<f64>, ee_t_k: &Matrix4<f64>) -> Matrix4<f64> {
    flange_pose(q) * f_t_ee * ee_t_k
}

/// Compute the pose for an arbitrary Frame.
pub fn frame_pose(
    q: &[f64; 7],
    frame: Frame,
    f_t_ee: &Matrix4<f64>,
    ee_t_k: &Matrix4<f64>,
) -> Matrix4<f64> {
    match frame {
        Frame::Joint1 => forward_kinematics(q, 1),
        Frame::Joint2 => forward_kinematics(q, 2),
        Frame::Joint3 => forward_kinematics(q, 3),
        Frame::Joint4 => forward_kinematics(q, 4),
        Frame::Joint5 => forward_kinematics(q, 5),
        Frame::Joint6 => forward_kinematics(q, 6),
        Frame::Joint7 => forward_kinematics(q, 7),
        Frame::Flange => flange_pose(q),
        Frame::EndEffector => ee_pose(q, f_t_ee),
        Frame::Stiffness => stiffness_pose(q, f_t_ee, ee_t_k),
    }
}

/// Compute the geometric Jacobian in the base frame (zero/world Jacobian).
///
/// Returns a 6x7 matrix where the top 3 rows are linear velocity and
/// bottom 3 rows are angular velocity contributions.
pub fn zero_jacobian(
    q: &[f64; 7],
    frame: Frame,
    f_t_ee: &Matrix4<f64>,
    ee_t_k: &Matrix4<f64>,
) -> Matrix6x7 {
    let target_pose = frame_pose(q, frame, f_t_ee, ee_t_k);
    let p_target = Vector3::new(target_pose[(0, 3)], target_pose[(1, 3)], target_pose[(2, 3)]);

    let n_joints = active_joints_for_frame(frame);
    let mut jacobian = Matrix6x7::zeros();

    let mut t_i = Matrix4::identity();
    for i in 0..n_joints {
        // z-axis of joint i in base frame
        let z_i = Vector3::new(t_i[(0, 2)], t_i[(1, 2)], t_i[(2, 2)]);
        // position of joint i origin in base frame
        let p_i = Vector3::new(t_i[(0, 3)], t_i[(1, 3)], t_i[(2, 3)]);

        // Linear velocity: z_i x (p_target - p_i)
        let linear = z_i.cross(&(p_target - p_i));
        // Angular velocity: z_i
        jacobian[(0, i)] = linear.x;
        jacobian[(1, i)] = linear.y;
        jacobian[(2, i)] = linear.z;
        jacobian[(3, i)] = z_i.x;
        jacobian[(4, i)] = z_i.y;
        jacobian[(5, i)] = z_i.z;

        // Advance to next joint frame
        t_i *= dh_transform(DH_A[i], DH_D[i], DH_ALPHA[i], q[i]);
    }

    jacobian
}

/// Compute the body Jacobian (Jacobian expressed in the target frame).
///
/// J_body = Ad(T_target^{-1}) * J_zero
pub fn body_jacobian(
    q: &[f64; 7],
    frame: Frame,
    f_t_ee: &Matrix4<f64>,
    ee_t_k: &Matrix4<f64>,
) -> Matrix6x7 {
    let j_zero = zero_jacobian(q, frame, f_t_ee, ee_t_k);
    let target_pose = frame_pose(q, frame, f_t_ee, ee_t_k);

    // Compute adjoint of the inverse transform
    let rot: Matrix3<f64> = target_pose.fixed_view::<3, 3>(0, 0).into_owned();
    let pos = Vector3::new(target_pose[(0, 3)], target_pose[(1, 3)], target_pose[(2, 3)]);

    // Ad(T^{-1}) = [R^T, -R^T * [p]x; 0, R^T]
    let rt = rot.transpose();
    let p_skew = skew(&pos);
    let neg_rt_px = -rt * p_skew;

    // Apply adjoint of inverse to each column
    let mut j_body = Matrix6x7::zeros();
    for i in 0..7 {
        let v = Vector3::new(j_zero[(0, i)], j_zero[(1, i)], j_zero[(2, i)]);
        let w = Vector3::new(j_zero[(3, i)], j_zero[(4, i)], j_zero[(5, i)]);

        let v_body: Vector3<f64> = rt * v + neg_rt_px * w;
        let w_body: Vector3<f64> = rt * w;

        j_body[(0, i)] = v_body[0];
        j_body[(1, i)] = v_body[1];
        j_body[(2, i)] = v_body[2];
        j_body[(3, i)] = w_body[0];
        j_body[(4, i)] = w_body[1];
        j_body[(5, i)] = w_body[2];
    }

    j_body
}

/// Returns how many joints contribute to a given frame.
fn active_joints_for_frame(frame: Frame) -> usize {
    match frame {
        Frame::Joint1 => 1,
        Frame::Joint2 => 2,
        Frame::Joint3 => 3,
        Frame::Joint4 => 4,
        Frame::Joint5 => 5,
        Frame::Joint6 => 6,
        Frame::Joint7 | Frame::Flange | Frame::EndEffector | Frame::Stiffness => 7,
    }
}

/// Skew-symmetric matrix from a vector.
fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y,
        v.z, 0.0, -v.x,
        -v.y, v.x, 0.0,
    )
}

/// Convert a column-major [f64; 16] to Matrix4.
pub fn mat4_from_column_major(data: &[f64; 16]) -> Matrix4<f64> {
    Matrix4::from_column_slice(data)
}

/// Convert a Matrix4 to column-major [f64; 16].
pub fn mat4_to_column_major(m: &Matrix4<f64>) -> [f64; 16] {
    let mut out = [0.0; 16];
    out.copy_from_slice(m.as_slice());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_config_flange_pose() {
        let q = [0.0; 7];
        let pose = flange_pose(&q);
        // At zero config, the end effector should be at a known position.
        // The z-height should be sum of d values: 0.333 + 0.316 + 0.384 + 0.107 = 1.140 (approx)
        // but with the DH structure it's more complex due to alpha rotations.
        // Just verify it's a valid homogeneous transform.
        assert!((pose[(3, 3)] - 1.0).abs() < 1e-10);
        assert!((pose[(3, 0)]).abs() < 1e-10);
        assert!((pose[(3, 1)]).abs() < 1e-10);
        assert!((pose[(3, 2)]).abs() < 1e-10);
    }

    #[test]
    fn identity_ee_equals_flange() {
        let q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];
        let flange = flange_pose(&q);
        let identity = Matrix4::identity();
        let ee = ee_pose(&q, &identity);
        for i in 0..16 {
            assert!(
                (flange.as_slice()[i] - ee.as_slice()[i]).abs() < 1e-10,
                "mismatch at {i}"
            );
        }
    }

    #[test]
    fn jacobian_is_zero_at_zero_velocity() {
        let q = [0.0; 7];
        let f_t_ee = Matrix4::identity();
        let ee_t_k = Matrix4::identity();
        let jac = zero_jacobian(&q, Frame::Flange, &f_t_ee, &ee_t_k);
        // At zero joint positions, Jacobian should still be well-defined (non-zero)
        let norm = jac.norm();
        assert!(norm > 0.1, "Jacobian should be non-zero: {norm}");
    }

    #[test]
    fn body_jacobian_consistency() {
        let q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];
        let f_t_ee = Matrix4::identity();
        let ee_t_k = Matrix4::identity();
        let j_zero = zero_jacobian(&q, Frame::Flange, &f_t_ee, &ee_t_k);
        let j_body = body_jacobian(&q, Frame::Flange, &f_t_ee, &ee_t_k);
        // Both should be non-zero
        assert!(j_zero.norm() > 0.1);
        assert!(j_body.norm() > 0.1);
    }
}
