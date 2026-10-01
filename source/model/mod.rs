//! Kinematic and dynamic model of the arm, built from the robot's URDF description.

mod chain;
mod dynamics;
mod kinematics;
mod spatial;
mod urdf;

#[cfg(test)]
mod tests;

use nalgebra::{Isometry3, SMatrix, SVector, Vector3};

pub use self::spatial::RigidBodyInertia;

use self::chain::KinematicChain;
use crate::constants::NUM_JOINTS;
use crate::errors::FrankaResult;
use crate::robot_state::RobotState;
use crate::types::{CartesianPose, Frame};

/// One value per joint, such as positions, velocities, or torques.
pub type JointVector = SVector<f64, NUM_JOINTS>;

/// 6×7 Jacobian whose rows are linear velocity (x, y, z) followed by angular velocity (x, y, z).
pub type Jacobian = SMatrix<f64, 6, NUM_JOINTS>;

/// 7×7 joint-space mass matrix.
pub type MassMatrix = SMatrix<f64, NUM_JOINTS, NUM_JOINTS>;

/// Kinematic and dynamic quantities of the arm.
///
/// `f_t_ee` is the end-effector pose in the flange frame, `ee_t_k` the stiffness pose in the
/// end-effector frame, and `payload` the total mass attached to the flange (end effector plus
/// load), expressed in the flange frame.
pub trait RobotModel {
    /// Returns the pose of `frame` in the base frame.
    fn pose(&self, frame: Frame, q: &JointVector, f_t_ee: &Isometry3<f64>, ee_t_k: &Isometry3<f64>) -> Isometry3<f64>;

    /// Returns the Jacobian of `frame`'s origin velocity and angular velocity in base-frame axes.
    fn zero_jacobian(&self, frame: Frame, q: &JointVector, f_t_ee: &Isometry3<f64>, ee_t_k: &Isometry3<f64>) -> Jacobian;

    /// Returns the Jacobian of `frame`'s twist expressed in `frame` itself.
    fn body_jacobian(&self, frame: Frame, q: &JointVector, f_t_ee: &Isometry3<f64>, ee_t_k: &Isometry3<f64>) -> Jacobian;

    /// Returns the joint-space mass matrix, in kg·m².
    fn mass(&self, q: &JointVector, payload: &RigidBodyInertia) -> MassMatrix;

    /// Returns the Coriolis and centrifugal joint torques, in N·m.
    fn coriolis(&self, q: &JointVector, dq: &JointVector, payload: &RigidBodyInertia) -> JointVector;

    /// Returns the joint torques that compensate `gravity` (base frame, m/s²), in N·m.
    fn gravity(&self, q: &JointVector, payload: &RigidBodyInertia, gravity: &Vector3<f64>) -> JointVector;

    /// Returns [`RobotModel::pose`] at the measured state.
    fn pose_from_state(&self, frame: Frame, state: &RobotState) -> Isometry3<f64> {
        let (q, f_t_ee, ee_t_k) = configuration_from_state(state);
        self.pose(frame, &q, &f_t_ee, &ee_t_k)
    }

    /// Returns [`RobotModel::zero_jacobian`] at the measured state.
    fn zero_jacobian_from_state(&self, frame: Frame, state: &RobotState) -> Jacobian {
        let (q, f_t_ee, ee_t_k) = configuration_from_state(state);
        self.zero_jacobian(frame, &q, &f_t_ee, &ee_t_k)
    }

    /// Returns [`RobotModel::body_jacobian`] at the measured state.
    fn body_jacobian_from_state(&self, frame: Frame, state: &RobotState) -> Jacobian {
        let (q, f_t_ee, ee_t_k) = configuration_from_state(state);
        self.body_jacobian(frame, &q, &f_t_ee, &ee_t_k)
    }

    /// Returns [`RobotModel::mass`] at the measured state with its configured total load.
    fn mass_from_state(&self, state: &RobotState) -> MassMatrix {
        self.mass(&JointVector::from(state.q), &state.total_load())
    }

    /// Returns [`RobotModel::coriolis`] at the measured state with its configured total load.
    fn coriolis_from_state(&self, state: &RobotState) -> JointVector {
        self.coriolis(&JointVector::from(state.q), &JointVector::from(state.dq), &state.total_load())
    }

    /// Returns [`RobotModel::gravity`] at the measured state, using the measured base
    /// acceleration `o_ddp_o` as gravity and the configured total load.
    fn gravity_from_state(&self, state: &RobotState) -> JointVector {
        self.gravity(
            &JointVector::from(state.q),
            &state.total_load(),
            &Vector3::from(state.o_ddp_o),
        )
    }
}

/// Extracts joint positions and the end-effector and stiffness frame offsets from `state`.
fn configuration_from_state(state: &RobotState) -> (JointVector, Isometry3<f64>, Isometry3<f64>) {
    (
        JointVector::from(state.q),
        CartesianPose::from_column_major(&state.f_t_ee).inner,
        CartesianPose::from_column_major(&state.ee_t_k).inner,
    )
}

/// Arm model computed from a URDF description.
#[derive(Debug, Clone, PartialEq)]
pub struct Model {
    /// Joint geometry and link inertias.
    chain: KinematicChain,
}

impl Model {
    /// Builds a model from a URDF document, such as the one returned by
    /// [`Robot::get_robot_model`](crate::robot::Robot::get_robot_model).
    ///
    /// # Errors
    /// Returns [`FrankaError::Model`](crate::errors::FrankaError::Model) if the URDF does not
    /// describe a serial arm of seven revolute joints ending in a `link8` flange.
    pub fn from_urdf(urdf: &str) -> FrankaResult<Self> {
        Ok(Self {
            chain: KinematicChain::from_urdf(urdf)?,
        })
    }
}

impl RobotModel for Model {
    fn pose(&self, frame: Frame, q: &JointVector, f_t_ee: &Isometry3<f64>, ee_t_k: &Isometry3<f64>) -> Isometry3<f64> {
        kinematics::frame_pose(&self.chain, frame, q, f_t_ee, ee_t_k)
    }

    fn zero_jacobian(&self, frame: Frame, q: &JointVector, f_t_ee: &Isometry3<f64>, ee_t_k: &Isometry3<f64>) -> Jacobian {
        kinematics::zero_jacobian(&self.chain, frame, q, f_t_ee, ee_t_k)
    }

    fn body_jacobian(&self, frame: Frame, q: &JointVector, f_t_ee: &Isometry3<f64>, ee_t_k: &Isometry3<f64>) -> Jacobian {
        kinematics::body_jacobian(&self.chain, frame, q, f_t_ee, ee_t_k)
    }

    fn mass(&self, q: &JointVector, payload: &RigidBodyInertia) -> MassMatrix {
        dynamics::mass_matrix(&self.chain, payload, q)
    }

    fn coriolis(&self, q: &JointVector, dq: &JointVector, payload: &RigidBodyInertia) -> JointVector {
        dynamics::inverse_dynamics(&self.chain, payload, q, dq, &JointVector::zeros(), &Vector3::zeros())
    }

    fn gravity(&self, q: &JointVector, payload: &RigidBodyInertia, gravity: &Vector3<f64>) -> JointVector {
        dynamics::gravity_torques(&self.chain, payload, q, gravity)
    }
}
