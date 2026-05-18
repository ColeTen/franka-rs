pub mod dynamics;
pub mod kinematics;

use nalgebra::Matrix4;

use crate::robot_state::RobotState;
use crate::types::Frame;

use self::dynamics::{default_link_params, LinkParams};
use self::kinematics::{mat4_from_column_major, mat4_to_column_major};

/// Robot kinematic and dynamic model.
///
/// Computes forward kinematics, Jacobians, mass matrix, Coriolis forces,
/// and gravity vector for the Franka Emika Panda/FR3 robot.
///
/// Uses the modified Denavit-Hartenberg parameters and identified link
/// inertial parameters.
pub struct Model {
    link_params: [LinkParams; 7],
    f_t_ee: Matrix4<f64>,
    ee_t_k: Matrix4<f64>,
}

impl Model {
    /// Create a new model with default parameters and identity frame transforms.
    pub fn new() -> Self {
        Self {
            link_params: default_link_params(),
            f_t_ee: Matrix4::identity(),
            ee_t_k: Matrix4::identity(),
        }
    }

    /// Create a model with custom frame transforms.
    pub fn with_frames(f_t_ee: &[f64; 16], ee_t_k: &[f64; 16]) -> Self {
        Self {
            link_params: default_link_params(),
            f_t_ee: mat4_from_column_major(f_t_ee),
            ee_t_k: mat4_from_column_major(ee_t_k),
        }
    }

    /// Set the flange-to-end-effector transform.
    pub fn set_f_t_ee(&mut self, f_t_ee: &[f64; 16]) {
        self.f_t_ee = mat4_from_column_major(f_t_ee);
    }

    /// Set the end-effector-to-stiffness-frame transform.
    pub fn set_ee_t_k(&mut self, ee_t_k: &[f64; 16]) {
        self.ee_t_k = mat4_from_column_major(ee_t_k);
    }

    // === Kinematics ===

    /// Compute the 4x4 pose (column-major) of the given frame.
    pub fn pose(&self, frame: Frame, q: &[f64; 7]) -> [f64; 16] {
        let m = kinematics::frame_pose(q, frame, &self.f_t_ee, &self.ee_t_k);
        mat4_to_column_major(&m)
    }

    /// Compute the pose from a full robot state.
    pub fn pose_from_state(&self, frame: Frame, state: &RobotState) -> [f64; 16] {
        self.pose(frame, &state.q)
    }

    /// Compute the 6x7 body Jacobian (column-major) for the given frame.
    ///
    /// The body Jacobian maps joint velocities to the twist expressed in the target frame.
    pub fn body_jacobian(&self, frame: Frame, q: &[f64; 7]) -> [f64; 42] {
        let j = kinematics::body_jacobian(q, frame, &self.f_t_ee, &self.ee_t_k);
        let mut out = [0.0; 42];
        out.copy_from_slice(j.as_slice());
        out
    }

    /// Compute the body Jacobian from a full robot state.
    pub fn body_jacobian_from_state(&self, frame: Frame, state: &RobotState) -> [f64; 42] {
        self.body_jacobian(frame, &state.q)
    }

    /// Compute the 6x7 zero (world/spatial) Jacobian (column-major) for the given frame.
    ///
    /// The zero Jacobian maps joint velocities to the twist expressed in the base frame.
    pub fn zero_jacobian(&self, frame: Frame, q: &[f64; 7]) -> [f64; 42] {
        let j = kinematics::zero_jacobian(q, frame, &self.f_t_ee, &self.ee_t_k);
        let mut out = [0.0; 42];
        out.copy_from_slice(j.as_slice());
        out
    }

    /// Compute the zero Jacobian from a full robot state.
    pub fn zero_jacobian_from_state(&self, frame: Frame, state: &RobotState) -> [f64; 42] {
        self.zero_jacobian(frame, &state.q)
    }

    // === Dynamics ===

    /// Compute the 7x7 mass (inertia) matrix (column-major).
    pub fn mass(&self, q: &[f64; 7], load_mass: f64, load_com: &[f64; 3], load_inertia: &[f64; 9]) -> [f64; 49] {
        dynamics::mass_matrix(q, &self.link_params, load_mass, load_com, load_inertia)
    }

    /// Compute the mass matrix from a full robot state.
    pub fn mass_from_state(&self, state: &RobotState) -> [f64; 49] {
        self.mass(&state.q, state.m_load, &state.f_x_cload, &state.i_load)
    }

    /// Compute the Coriolis/centrifugal force vector (7 elements).
    pub fn coriolis(
        &self,
        q: &[f64; 7],
        dq: &[f64; 7],
        load_mass: f64,
        load_com: &[f64; 3],
        load_inertia: &[f64; 9],
    ) -> [f64; 7] {
        dynamics::coriolis_vector(q, dq, &self.link_params, load_mass, load_com, load_inertia)
    }

    /// Compute Coriolis from a full robot state.
    pub fn coriolis_from_state(&self, state: &RobotState) -> [f64; 7] {
        self.coriolis(
            &state.q,
            &state.dq,
            state.m_load,
            &state.f_x_cload,
            &state.i_load,
        )
    }

    /// Compute the gravity torque vector (7 elements).
    ///
    /// `gravity_earth` is the gravity vector in the base frame, default [0, 0, -9.81].
    pub fn gravity(
        &self,
        q: &[f64; 7],
        load_mass: f64,
        load_com: &[f64; 3],
        gravity_earth: &[f64; 3],
    ) -> [f64; 7] {
        dynamics::gravity_vector(q, &self.link_params, load_mass, load_com, gravity_earth)
    }

    /// Compute gravity from a full robot state with default Earth gravity.
    pub fn gravity_from_state(&self, state: &RobotState) -> [f64; 7] {
        self.gravity(&state.q, state.m_load, &state.f_x_cload, &[0.0, 0.0, -9.81])
    }
}

impl Default for Model {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_pose_consistency() {
        let model = Model::new();
        let q = [0.0; 7];

        let flange = model.pose(Frame::Flange, &q);
        let ee = model.pose(Frame::EndEffector, &q);

        // With identity F_T_EE, flange == end effector
        for i in 0..16 {
            assert!(
                (flange[i] - ee[i]).abs() < 1e-10,
                "flange and EE should match with identity F_T_EE at index {i}"
            );
        }
    }

    #[test]
    fn model_jacobian_nonzero() {
        let model = Model::new();
        let q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7];

        let j = model.zero_jacobian(Frame::EndEffector, &q);
        let norm: f64 = j.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(norm > 0.1, "Jacobian should be non-zero");
    }

    #[test]
    fn model_gravity_nonzero() {
        let model = Model::new();
        let q = [0.0; 7];
        let g = model.gravity(&q, 0.0, &[0.0; 3], &[0.0, 0.0, -9.81]);
        let norm: f64 = g.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(norm > 0.1, "gravity should be non-zero: {g:?}");
    }
}
