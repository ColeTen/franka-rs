//! Rigid-body dynamics of a [`KinematicChain`]: inverse dynamics, gravity torques, and mass matrix.
//!
//! All functions take the payload rigidly attached to the flange, expressed in the flange frame.

use nalgebra::{Isometry3, Vector3};

use super::chain::KinematicChain;
use super::spatial::{RigidBodyInertia, SpatialForce, SpatialMotion};
use super::{JointVector, MassMatrix};
use crate::constants::NUM_JOINTS;

/// Returns the pose of every joint frame in its parent joint's frame at joint positions `q`.
fn parent_from_joint_poses(chain: &KinematicChain, q: &JointVector) -> [Isometry3<f64>; NUM_JOINTS] {
    std::array::from_fn(|joint_index| chain.parent_from_joint(joint_index, q[joint_index]))
}

/// Returns the base acceleration that models gravity as a fictitious upward acceleration.
fn base_acceleration(gravity: &Vector3<f64>) -> SpatialMotion {
    SpatialMotion {
        linear: -gravity,
        angular: Vector3::zeros(),
    }
}

/// Propagates joint forces from tip to base and returns each joint's torque.
fn project_forces_to_torques(
    chain: &KinematicChain,
    parent_from_joint: &[Isometry3<f64>; NUM_JOINTS],
    mut forces: [SpatialForce; NUM_JOINTS],
) -> JointVector {
    let mut torques = JointVector::zeros();
    for joint_index in (0..NUM_JOINTS).rev() {
        torques[joint_index] = chain.joints[joint_index].axis.dot(&forces[joint_index].angular);
        if joint_index > 0 {
            forces[joint_index - 1] =
                forces[joint_index - 1] + forces[joint_index].act(&parent_from_joint[joint_index]);
        }
    }
    torques
}

/// Returns the joint torques that produce accelerations `ddq` at positions `q` and velocities
/// `dq` under base-frame `gravity`.
pub fn inverse_dynamics(
    chain: &KinematicChain,
    payload: &RigidBodyInertia,
    q: &JointVector,
    dq: &JointVector,
    ddq: &JointVector,
    gravity: &Vector3<f64>,
) -> JointVector {
    let inertias = chain.inertias_with_payload(payload);
    let parent_from_joint = parent_from_joint_poses(chain, q);
    let mut velocity = SpatialMotion::zero();
    let mut acceleration = base_acceleration(gravity);
    let mut forces = [SpatialForce {
        linear: Vector3::zeros(),
        angular: Vector3::zeros(),
    }; NUM_JOINTS];
    for joint_index in 0..NUM_JOINTS {
        let axis = &chain.joints[joint_index].axis;
        let joint_velocity = SpatialMotion::revolute(axis, dq[joint_index]);
        velocity = joint_velocity + velocity.act_inverse(&parent_from_joint[joint_index]);
        acceleration = velocity.cross_motion(&joint_velocity)
            + SpatialMotion::revolute(axis, ddq[joint_index])
            + acceleration.act_inverse(&parent_from_joint[joint_index]);
        let momentum = inertias[joint_index].apply(&velocity);
        forces[joint_index] =
            inertias[joint_index].apply(&acceleration) + velocity.cross_force(&momentum);
    }
    project_forces_to_torques(chain, &parent_from_joint, forces)
}

/// Returns the joint torques that hold the arm static at positions `q` under base-frame `gravity`.
pub fn gravity_torques(
    chain: &KinematicChain,
    payload: &RigidBodyInertia,
    q: &JointVector,
    gravity: &Vector3<f64>,
) -> JointVector {
    let inertias = chain.inertias_with_payload(payload);
    let parent_from_joint = parent_from_joint_poses(chain, q);
    let mut acceleration = base_acceleration(gravity);
    let forces = std::array::from_fn(|joint_index| {
        acceleration = acceleration.act_inverse(&parent_from_joint[joint_index]);
        inertias[joint_index].apply(&acceleration)
    });
    project_forces_to_torques(chain, &parent_from_joint, forces)
}

/// Returns the joint-space mass matrix at positions `q`.
pub fn mass_matrix(chain: &KinematicChain, payload: &RigidBodyInertia, q: &JointVector) -> MassMatrix {
    let mut composite_inertias = chain.inertias_with_payload(payload);
    let parent_from_joint = parent_from_joint_poses(chain, q);
    let mut subtree_forces = [SpatialForce {
        linear: Vector3::zeros(),
        angular: Vector3::zeros(),
    }; NUM_JOINTS];
    let mut mass = MassMatrix::zeros();
    for joint_index in (0..NUM_JOINTS).rev() {
        let axis = &chain.joints[joint_index].axis;
        subtree_forces[joint_index] =
            composite_inertias[joint_index].apply(&SpatialMotion::revolute(axis, 1.0));
        for column in joint_index..NUM_JOINTS {
            mass[(joint_index, column)] = axis.dot(&subtree_forces[column].angular);
        }
        if joint_index > 0 {
            let to_parent = &parent_from_joint[joint_index];
            composite_inertias[joint_index - 1] = composite_inertias[joint_index - 1]
                .combine(&composite_inertias[joint_index].transformed(to_parent));
            for force in &mut subtree_forces[joint_index..] {
                *force = force.act(to_parent);
            }
        }
    }
    mass.fill_lower_triangle_with_upper_triangle();
    mass
}
