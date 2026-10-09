//! Rigid-body dynamics of a [`KinematicChain`]: inverse dynamics, gravity torques, mass matrix, and
//! Coriolis matrix.
//!
//! All functions take the payload rigidly attached to the flange, expressed in the flange frame.

use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};

use super::chain::KinematicChain;
use super::spatial::{RigidBodyInertia, SpatialForce, SpatialMotion};
use super::{CoriolisMatrix, JointVector, MassMatrix};
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

/// Returns the Coriolis matrix C(q, dq) computed by pinocchio's `computeCoriolisMatrix` algorithm,
/// so that `C * dq` is the Coriolis and centrifugal torque vector.
///
/// Forward pass, in the world frame: each joint's motion subspace column `J`, its derivative
/// `dJ = v × J`, the link inertia `Y` and the matrix `B = Y.variation(v/2) + force_cross(Y v / 2)`.
/// Backward pass: the rows of C from the composite inertias and `B` of each subtree.
pub fn coriolis_matrix(chain: &KinematicChain, payload: &RigidBodyInertia, q: &JointVector, dq: &JointVector) -> CoriolisMatrix {
    let inertias = chain.inertias_with_payload(payload);
    let parent_from_joint = parent_from_joint_poses(chain, q);

    let mut columns = [Vector6::zeros(); NUM_JOINTS];
    let mut column_derivatives = [Vector6::zeros(); NUM_JOINTS];
    let mut composite_inertias = [Matrix6::zeros(); NUM_JOINTS];
    let mut velocity_product_matrices = [Matrix6::zeros(); NUM_JOINTS];
    let mut world_pose = Isometry3::identity();
    let mut velocity = SpatialMotion::zero();
    for joint_index in 0..NUM_JOINTS {
        world_pose *= parent_from_joint[joint_index];
        let column = SpatialMotion::revolute(&chain.joints[joint_index].axis, 1.0).act(&world_pose);
        velocity = SpatialMotion {
            linear: velocity.linear + column.linear * dq[joint_index],
            angular: velocity.angular + column.angular * dq[joint_index],
        };
        let world_inertia = inertias[joint_index].transformed(&world_pose);
        let momentum = world_inertia.apply(&velocity);
        let scaled_by_half = |value: Vector3<f64>| value * 0.5;
        columns[joint_index] = column.to_vector();
        column_derivatives[joint_index] = velocity.cross_motion(&column).to_vector();
        composite_inertias[joint_index] = world_inertia.spatial_matrix();
        velocity_product_matrices[joint_index] = world_inertia.variation(&SpatialMotion {
            linear: scaled_by_half(velocity.linear),
            angular: scaled_by_half(velocity.angular),
        }) + SpatialForce {
            linear: scaled_by_half(momentum.linear),
            angular: scaled_by_half(momentum.angular),
        }
        .cross_matrix();
    }

    let mut coriolis = CoriolisMatrix::zeros();
    let mut force_derivatives = [Vector6::zeros(); NUM_JOINTS];
    for joint_index in (0..NUM_JOINTS).rev() {
        let column = &columns[joint_index];
        force_derivatives[joint_index] = composite_inertias[joint_index] * column_derivatives[joint_index]
            + velocity_product_matrices[joint_index] * column;
        for other in joint_index..NUM_JOINTS {
            coriolis[(joint_index, other)] = column.dot(&force_derivatives[other]);
        }
        let momentum_column = composite_inertias[joint_index] * column;
        let column_times_b = column.transpose() * velocity_product_matrices[joint_index];
        for ancestor in 0..joint_index {
            coriolis[(joint_index, ancestor)] = momentum_column.dot(&column_derivatives[ancestor])
                + (column_times_b * columns[ancestor])[0];
        }
        if joint_index > 0 {
            let (lower, upper) = composite_inertias.split_at_mut(joint_index);
            lower[joint_index - 1] += upper[0];
            let (lower, upper) = velocity_product_matrices.split_at_mut(joint_index);
            lower[joint_index - 1] += upper[0];
        }
    }
    coriolis
}
