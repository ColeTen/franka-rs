use nalgebra::{Matrix3, Matrix4, Vector3};

use super::kinematics::{DH_A, DH_ALPHA, DH_D};
use crate::constants::NUM_JOINTS;

/// Link inertial parameters for the Franka Emika Panda/FR3.
///
/// Each link has: mass, center of mass (in link frame), and inertia tensor (3x3, in CoM frame).
/// These are approximate values from the official Franka documentation and identified parameters.
#[derive(Debug, Clone)]
pub struct LinkParams {
    pub mass: f64,
    pub com: Vector3<f64>,
    pub inertia: Matrix3<f64>,
}

/// Default link parameters for the Franka Emika Panda/FR3 (approximate).
///
/// Source: Official Franka documentation and system identification papers.
pub fn default_link_params() -> [LinkParams; 7] {
    [
        // Link 1
        LinkParams {
            mass: 4.970684,
            com: Vector3::new(0.003875, 0.002081, -0.04762),
            inertia: Matrix3::new(
                0.70337, -0.000139, 0.006772,
                -0.000139, 0.70661, 0.019169,
                0.006772, 0.019169, 0.009117,
            ),
        },
        // Link 2
        LinkParams {
            mass: 0.646926,
            com: Vector3::new(-0.003141, -0.02872, 0.003495),
            inertia: Matrix3::new(
                0.007962, -0.003925, 0.010254,
                -0.003925, 0.02811, 0.000704,
                0.010254, 0.000704, 0.025995,
            ),
        },
        // Link 3
        LinkParams {
            mass: 3.228604,
            com: Vector3::new(0.02723, 0.039252, -0.066502),
            inertia: Matrix3::new(
                0.037242, -0.004761, -0.011396,
                -0.004761, 0.036155, -0.012805,
                -0.011396, -0.012805, 0.010830,
            ),
        },
        // Link 4
        LinkParams {
            mass: 3.587895,
            com: Vector3::new(-0.05317, 0.104419, 0.027454),
            inertia: Matrix3::new(
                0.025853, 0.007796, -0.001332,
                0.007796, 0.019552, 0.008641,
                -0.001332, 0.008641, 0.028323,
            ),
        },
        // Link 5
        LinkParams {
            mass: 1.225946,
            com: Vector3::new(-0.011953, 0.041065, -0.038437),
            inertia: Matrix3::new(
                0.035549, -0.002117, -0.004037,
                -0.002117, 0.029474, 0.000229,
                -0.004037, 0.000229, 0.008627,
            ),
        },
        // Link 6
        LinkParams {
            mass: 1.666555,
            com: Vector3::new(0.060149, -0.014117, -0.010517),
            inertia: Matrix3::new(
                0.001964, 0.000109, -0.001158,
                0.000109, 0.004354, 0.000341,
                -0.001158, 0.000341, 0.005433,
            ),
        },
        // Link 7
        LinkParams {
            mass: 0.735522,
            com: Vector3::new(0.010517, -0.004252, 0.061597),
            inertia: Matrix3::new(
                0.012516, -0.000428, -0.001196,
                -0.000428, 0.010027, -0.000741,
                -0.001196, -0.000741, 0.004815,
            ),
        },
    ]
}

/// Compute a single Modified DH transform (same as kinematics, duplicated to keep module standalone).
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

/// Compute the 7x7 mass (inertia) matrix using the Composite Rigid Body Algorithm.
///
/// M(q) such that τ = M(q) * ddq + C(q, dq) * dq + g(q)
pub fn mass_matrix(
    q: &[f64; 7],
    link_params: &[LinkParams; 7],
    load_mass: f64,
    load_com: &[f64; 3],
    load_inertia: &[f64; 9],
) -> [f64; 49] {
    // Compute transforms from base to each link frame
    let mut transforms = [Matrix4::identity(); 7];
    let mut t = Matrix4::identity();
    for i in 0..7 {
        t *= dh_transform(DH_A[i], DH_D[i], DH_ALPHA[i], q[i]);
        transforms[i] = t;
    }

    // Composite inertias in spatial form (6x6 would be ideal, but we use 3x3 rotational + mass)
    // We use the CRBA: compute composite inertia from tip to base.
    let mut mass = [[0.0f64; 7]; 7];

    // For each column j of M (virtual unit acceleration at joint j):
    for j in 0..NUM_JOINTS {
        // z-axis of joint j in base frame
        let t_j = if j == 0 {
            Matrix4::identity()
        } else {
            transforms[j - 1]
        };
        let z_j = Vector3::new(t_j[(0, 2)], t_j[(1, 2)], t_j[(2, 2)]);
        let p_j = Vector3::new(t_j[(0, 3)], t_j[(1, 3)], t_j[(2, 3)]);

        // Compute M[i][j] by projecting forces from unit accel at joint j onto joint i
        for i in 0..=j {
            let t_i = if i == 0 {
                Matrix4::identity()
            } else {
                transforms[i - 1]
            };
            let z_i = Vector3::new(t_i[(0, 2)], t_i[(1, 2)], t_i[(2, 2)]);
            let p_i = Vector3::new(t_i[(0, 3)], t_i[(1, 3)], t_i[(2, 3)]);

            let mut tau_i = Vector3::zeros();

            for k in i.max(j)..NUM_JOINTS {
                let rot_k: Matrix3<f64> = transforms[k].fixed_view::<3, 3>(0, 0).into_owned();
                let pos_k = Vector3::new(
                    transforms[k][(0, 3)],
                    transforms[k][(1, 3)],
                    transforms[k][(2, 3)],
                );

                let m_k = link_params[k].mass;
                let com_world = pos_k + rot_k * link_params[k].com;
                let inertia_world = rot_k * link_params[k].inertia * rot_k.transpose();

                let r_from_j = com_world - p_j;
                let linear_accel_j = z_j.cross(&r_from_j);

                let r_from_i = com_world - p_i;

                // Force on link k due to unit accel at j
                let fk = m_k * linear_accel_j;
                // Torque on link k
                let tau_k = inertia_world * z_j + m_k * r_from_j.cross(&linear_accel_j);

                tau_i += tau_k + r_from_i.cross(&fk);

                // Handle load on last link
                if k == NUM_JOINTS - 1 && load_mass > 0.0 {
                    let load_com_vec = Vector3::new(load_com[0], load_com[1], load_com[2]);
                    let load_com_world = pos_k + rot_k * load_com_vec;
                    let load_inertia_mat = Matrix3::from_column_slice(load_inertia);
                    let load_inertia_world = rot_k * load_inertia_mat * rot_k.transpose();

                    let r_load_from_j = load_com_world - p_j;
                    let linear_accel_load = z_j.cross(&r_load_from_j);
                    let r_load_from_i = load_com_world - p_i;

                    let fk_load = load_mass * linear_accel_load;
                    let tau_k_load = load_inertia_world * z_j
                        + load_mass * r_load_from_j.cross(&linear_accel_load);

                    tau_i += tau_k_load + r_load_from_i.cross(&fk_load);
                }
            }

            mass[i][j] = z_i.dot(&tau_i);
            mass[j][i] = mass[i][j];
        }
    }

    // Flatten to column-major [f64; 49]
    let mut result = [0.0; 49];
    for col in 0..7 {
        for row in 0..7 {
            result[col * 7 + row] = mass[row][col];
        }
    }
    result
}

/// Compute the gravity torque vector.
///
/// g(q) = vector of gravity-induced joint torques.
pub fn gravity_vector(
    q: &[f64; 7],
    link_params: &[LinkParams; 7],
    load_mass: f64,
    load_com: &[f64; 3],
    gravity: &[f64; 3],
) -> [f64; 7] {
    let g = Vector3::new(gravity[0], gravity[1], gravity[2]);

    // Compute transforms
    let mut transforms = [Matrix4::identity(); 7];
    let mut t = Matrix4::identity();
    for i in 0..7 {
        t *= dh_transform(DH_A[i], DH_D[i], DH_ALPHA[i], q[i]);
        transforms[i] = t;
    }

    let mut result = [0.0; 7];

    for i in 0..NUM_JOINTS {
        let t_i = if i == 0 {
            Matrix4::identity()
        } else {
            transforms[i - 1]
        };
        let z_i = Vector3::new(t_i[(0, 2)], t_i[(1, 2)], t_i[(2, 2)]);
        let p_i = Vector3::new(t_i[(0, 3)], t_i[(1, 3)], t_i[(2, 3)]);

        let mut tau_gravity = 0.0;

        for k in i..NUM_JOINTS {
            let rot_k: Matrix3<f64> = transforms[k].fixed_view::<3, 3>(0, 0).into_owned();
            let pos_k = Vector3::new(
                transforms[k][(0, 3)],
                transforms[k][(1, 3)],
                transforms[k][(2, 3)],
            );

            let com_world = pos_k + rot_k * link_params[k].com;
            let r = com_world - p_i;
            let force = link_params[k].mass * g;
            tau_gravity += z_i.dot(&r.cross(&force));

            // Add load contribution on last link
            if k == NUM_JOINTS - 1 && load_mass > 0.0 {
                let load_com_vec = Vector3::new(load_com[0], load_com[1], load_com[2]);
                let load_com_world = pos_k + rot_k * load_com_vec;
                let r_load = load_com_world - p_i;
                let force_load = load_mass * g;
                tau_gravity += z_i.dot(&r_load.cross(&force_load));
            }
        }

        result[i] = tau_gravity;
    }

    result
}

/// Compute the Coriolis/centrifugal force vector using Christoffel symbols.
///
/// c(q, dq) = C(q, dq) * dq where C_ijk = 0.5 * (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)
pub fn coriolis_vector(
    q: &[f64; 7],
    dq: &[f64; 7],
    link_params: &[LinkParams; 7],
    load_mass: f64,
    load_com: &[f64; 3],
    load_inertia: &[f64; 9],
) -> [f64; 7] {
    // Numerical differentiation of the mass matrix to get Christoffel symbols
    let eps = 1e-8;
    let m0 = mass_matrix(q, link_params, load_mass, load_com, load_inertia);

    let mut dm_dq = [[[0.0f64; 7]; 7]; 7]; // dm_dq[k][i][j] = dM_ij/dq_k

    for k in 0..7 {
        let mut q_plus = *q;
        q_plus[k] += eps;
        let m_plus = mass_matrix(&q_plus, link_params, load_mass, load_com, load_inertia);

        for i in 0..7 {
            for j in 0..7 {
                dm_dq[k][i][j] = (m_plus[j * 7 + i] - m0[j * 7 + i]) / eps;
            }
        }
    }

    // Compute c_i = sum_j sum_k C_ijk * dq_j * dq_k
    // where C_ijk = 0.5 * (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)
    let mut c = [0.0; 7];
    for i in 0..7 {
        for j in 0..7 {
            for k in 0..7 {
                let christoffel =
                    0.5 * (dm_dq[k][i][j] + dm_dq[j][i][k] - dm_dq[i][j][k]);
                c[i] += christoffel * dq[j] * dq[k];
            }
        }
    }

    c
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gravity_vector_at_zero() {
        let q = [0.0; 7];
        let params = default_link_params();
        let g = gravity_vector(&q, &params, 0.0, &[0.0; 3], &[0.0, 0.0, -9.81]);
        // Gravity vector should not be all zeros (robot has mass)
        let norm: f64 = g.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(norm > 0.1, "gravity vector should be non-zero: {g:?}");
    }

    #[test]
    fn mass_matrix_symmetry() {
        let q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];
        let params = default_link_params();
        let m = mass_matrix(&q, &params, 0.0, &[0.0; 3], &[0.0; 9]);

        for i in 0..7 {
            for j in 0..7 {
                let m_ij = m[j * 7 + i];
                let m_ji = m[i * 7 + j];
                assert!(
                    (m_ij - m_ji).abs() < 1e-6,
                    "M[{i}][{j}] = {m_ij}, M[{j}][{i}] = {m_ji}"
                );
            }
        }
    }

    #[test]
    fn mass_matrix_positive_diagonal() {
        let q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];
        let params = default_link_params();
        let m = mass_matrix(&q, &params, 0.0, &[0.0; 3], &[0.0; 9]);

        for i in 0..7 {
            let m_ii = m[i * 7 + i];
            assert!(m_ii > 0.0, "M[{i}][{i}] = {m_ii} should be positive");
        }
    }

    #[test]
    fn coriolis_zero_at_zero_velocity() {
        let q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];
        let dq = [0.0; 7];
        let params = default_link_params();
        let c = coriolis_vector(&q, &dq, &params, 0.0, &[0.0; 3], &[0.0; 9]);

        for (i, &ci) in c.iter().enumerate() {
            assert!(
                ci.abs() < 1e-6,
                "c[{i}] = {ci} should be zero at zero velocity"
            );
        }
    }
}
