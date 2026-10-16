//! Rigid-body dynamics of a [`KinematicChain`]: inverse dynamics, gravity torques, mass matrix,
//! mass matrix inverse, and Coriolis matrix.
//!
//! All functions take the payload rigidly attached to the flange, expressed in the flange frame.

use nalgebra::{
    Isometry3, Matrix3, Matrix6, RowVector6, Translation3, Unit, UnitQuaternion, Vector3, Vector6, LU, U6,
};

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

/// Number of parent-child joint pairs in the chain.
const NUM_COUPLINGS: usize = NUM_JOINTS - 1;

/// Index of the angular z component in a `[linear; angular]` spatial vector.
const ANGULAR_Z: usize = 5;

/// Returns a rotation that takes the z axis onto `axis`.
fn axis_alignment(axis: &Unit<Vector3<f64>>) -> UnitQuaternion<f64> {
    // `rotation_between_axis` has no unique answer, and returns `None`, only when `axis` is -z;
    // a half turn about x then does.
    UnitQuaternion::rotation_between_axis(&Vector3::z_axis(), axis)
        .unwrap_or_else(|| UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI))
}

/// Returns the 6×6 matrix that applies `rotation` to both halves of a spatial vector.
fn spatial_rotation(rotation: &Matrix3<f64>) -> Matrix6<f64> {
    let mut matrix = Matrix6::zeros();
    matrix.fixed_view_mut::<3, 3>(0, 0).copy_from(rotation);
    matrix.fixed_view_mut::<3, 3>(3, 3).copy_from(rotation);
    matrix
}

/// Returns the 6×6 matrix that re-refers a motion from a point to the point at `offset` from it.
fn motion_shift(offset: &Vector3<f64>) -> Matrix6<f64> {
    let mut matrix = Matrix6::identity();
    matrix.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-offset.cross_matrix()));
    matrix
}

/// Returns `force` re-referred from the point at `offset` back to the original point.
fn shift_force_back(force: &Vector6<f64>, offset: &Vector3<f64>) -> Vector6<f64> {
    let linear = force.fixed_rows::<3>(0).into_owned();
    let mut shifted = *force;
    let mut angular = shifted.fixed_rows_mut::<3>(3);
    angular += offset.cross(&linear);
    shifted
}

/// Returns `vector` with `rotation` applied to both of its halves.
fn rotate_spatial(rotation: &Matrix3<f64>, vector: &Vector6<f64>) -> Vector6<f64> {
    let mut rotated = Vector6::zeros();
    rotated.fixed_rows_mut::<3>(0).copy_from(&(rotation * vector.fixed_rows::<3>(0)));
    rotated.fixed_rows_mut::<3>(3).copy_from(&(rotation * vector.fixed_rows::<3>(3)));
    rotated
}

/// Rotation about the z axis, stored as the cosine and sine of its angle.
#[derive(Debug, Clone, Copy)]
struct ZRotation {
    /// Cosine of the angle.
    cos: f64,
    /// Sine of the angle.
    sin: f64,
}

impl ZRotation {
    /// Returns the rotation by `angle` radians.
    fn new(angle: f64) -> Self {
        let (sin, cos) = angle.sin_cos();
        Self { cos, sin }
    }

    /// Returns the opposite rotation.
    fn inverse(self) -> Self {
        Self { cos: self.cos, sin: -self.sin }
    }

    /// Returns `vector` with both halves rotated about z by this rotation's angle.
    fn apply_spatial(self, vector: &Vector6<f64>) -> Vector6<f64> {
        let mut rotated = *vector;
        for first in [0, 3] {
            rotated[first] = self.cos * vector[first] - self.sin * vector[first + 1];
            rotated[first + 1] = self.sin * vector[first] + self.cos * vector[first + 1];
        }
        rotated
    }

    /// Returns `diag(R, R) · matrix` for this rotation `R`.
    fn apply_rows(self, matrix: &Matrix6<f64>) -> Matrix6<f64> {
        let mut rotated = *matrix;
        for first in [0, 3] {
            let (x_row, y_row) = (matrix.row(first), matrix.row(first + 1));
            rotated.set_row(first, &(x_row * self.cos - y_row * self.sin));
            rotated.set_row(first + 1, &(x_row * self.sin + y_row * self.cos));
        }
        rotated
    }

    /// Returns `matrix · diag(R, R)` for this rotation `R`.
    fn apply_columns(self, matrix: &Matrix6<f64>) -> Matrix6<f64> {
        let mut rotated = *matrix;
        for first in [0, 3] {
            let (x_column, y_column) = (matrix.column(first), matrix.column(first + 1));
            rotated.set_column(first, &(x_column * self.cos + y_column * self.sin));
            rotated.set_column(first + 1, &(y_column * self.cos - x_column * self.sin));
        }
        rotated
    }
}

/// Inertia terms of one link about its joint origin, in a frame whose z axis is the joint axis.
#[derive(Debug, Clone, Copy, PartialEq)]
struct AxisInertia {
    /// Spatial inverse inertia, mapping a force to the acceleration it gives the link at rest.
    inverse: Matrix6<f64>,
    /// Row of the spatial inertia that gives the torque about the joint axis.
    axis_row: RowVector6<f64>,
    /// Reciprocal of the link's moment of inertia about the joint axis.
    inverse_axis_inertia: f64,
}

impl AxisInertia {
    /// Returns the terms of `inertia`, which must be expressed in a frame whose z axis is the joint axis.
    fn new(inertia: &RigidBodyInertia) -> Self {
        let axis_row = inertia.spatial_matrix().row(ANGULAR_Z).into_owned();
        Self {
            inverse: inertia.inverse_spatial_matrix(),
            axis_row,
            inverse_axis_inertia: 1.0 / axis_row[ANGULAR_Z],
        }
    }

    /// Returns `(1 − H (Hᵀ I H)⁻¹ Hᵀ I) · matrix` for the joint axis `H` and this inertia `I`.
    ///
    /// The product `H (Hᵀ I H)⁻¹ Hᵀ I` is zero outside its angular z row, so only that row changes.
    fn remove_axis_component(&self, matrix: &Matrix6<f64>) -> Matrix6<f64> {
        let mut result = *matrix;
        let correction = self.axis_row * matrix * self.inverse_axis_inertia;
        let mut axis_row = result.row_mut(ANGULAR_Z);
        axis_row -= correction;
        result
    }
}

/// Configuration-independent terms of [`mass_matrix_inverse`], computed once per chain.
///
/// Each link's terms are expressed in its joint frame turned so that the joint axis is z (the link
/// frame), and each joint's zero-angle frame is turned the same way. Index `k` of every coupling
/// array relates link `k` to joint `k + 1`.
#[derive(Debug, Clone, PartialEq)]
pub struct MassMatrixInverseConstants {
    /// Pose of the last joint's frame in the last link frame.
    last_link_from_joint: Isometry3<f64>,
    /// Inertia terms of every link except the last, whose terms depend on the payload.
    link_inertias: [AxisInertia; NUM_COUPLINGS],
    /// Rotation from joint `k + 1`'s zero-angle frame to link `k`'s frame.
    child_rotations: [Matrix3<f64>; NUM_COUPLINGS],
    /// Origin of joint `k + 1` in link `k`'s frame.
    child_offsets: [Vector3<f64>; NUM_COUPLINGS],
    /// Inverse inertia of link `k` about joint `k + 1`'s origin, in joint `k + 1`'s zero-angle frame.
    parent_inverse_inertias: [Matrix6<f64>; NUM_COUPLINGS],
    /// Map from a force on link `k` (link frame) to the acceleration it gives link `k` at joint
    /// `k + 1`'s origin, in joint `k + 1`'s zero-angle frame.
    shifted_parent_inverse_inertias: [Matrix6<f64>; NUM_COUPLINGS],
    /// `(H (Hᵀ I H)⁻¹ Hᵀ − I⁻¹)` of link `k` times the map from a force at joint `k + 1` (zero-angle
    /// frame) to the same force at link `k`'s origin (link frame).
    child_couplings: [Matrix6<f64>; NUM_COUPLINGS],
}

impl MassMatrixInverseConstants {
    /// Computes the constants of `chain`.
    ///
    /// # Panics
    /// Panics if a link has a singular rotational inertia, which no physical link has.
    pub fn new(chain: &KinematicChain) -> Self {
        let alignments = chain.joints.each_ref().map(|joint| axis_alignment(&joint.axis));
        let link_from_joint =
            |joint_index: usize| Isometry3::from_parts(Translation3::identity(), alignments[joint_index].inverse());
        let link_inertias: [AxisInertia; NUM_COUPLINGS] = std::array::from_fn(|link_index| {
            AxisInertia::new(&chain.joints[link_index].inertia.transformed(&link_from_joint(link_index)))
        });
        let child_rotations: [Matrix3<f64>; NUM_COUPLINGS] = std::array::from_fn(|link_index| {
            (alignments[link_index].inverse() * chain.joints[link_index + 1].placement.rotation * alignments[link_index + 1])
                .to_rotation_matrix()
                .into_inner()
        });
        let child_offsets: [Vector3<f64>; NUM_COUPLINGS] = std::array::from_fn(|link_index| {
            alignments[link_index].inverse() * chain.joints[link_index + 1].placement.translation.vector
        });
        let force_to_parent =
            |link_index: usize| motion_shift(&child_offsets[link_index]).transpose() * spatial_rotation(&child_rotations[link_index]);
        let shifted_parent_inverse_inertias: [Matrix6<f64>; NUM_COUPLINGS] = std::array::from_fn(|link_index| {
            force_to_parent(link_index).transpose() * link_inertias[link_index].inverse
        });
        Self {
            last_link_from_joint: link_from_joint(NUM_JOINTS - 1),
            link_inertias,
            child_rotations,
            child_offsets,
            parent_inverse_inertias: std::array::from_fn(|link_index| {
                shifted_parent_inverse_inertias[link_index] * force_to_parent(link_index)
            }),
            shifted_parent_inverse_inertias,
            child_couplings: std::array::from_fn(|link_index| {
                let link = &link_inertias[link_index];
                let mut projection_minus_inverse = -link.inverse;
                projection_minus_inverse[(ANGULAR_Z, ANGULAR_Z)] += link.inverse_axis_inertia;
                projection_minus_inverse * force_to_parent(link_index)
            }),
        }
    }
}

/// Terms of the factorization that turn with the joints. Block `i` of the factorization is
/// expressed in link `i`'s frame, so each link's own terms are constant and only the terms coupling
/// neighbouring links depend on the joint angles.
mod block_frame {
    use nalgebra::{Matrix6, Vector3, Vector6};

    use super::{rotate_spatial, shift_force_back, MassMatrixInverseConstants, ZRotation};

    /// Returns link `k`'s inverse inertia about joint `k + 1`'s origin in block `k + 1`'s frame.
    pub(super) fn parent_inverse_inertia(constants: &MassMatrixInverseConstants, link_index: usize, rotations: &[ZRotation]) -> Matrix6<f64> {
        let rotation = rotations[link_index + 1];
        rotation.inverse().apply_rows(&rotation.apply_columns(&constants.parent_inverse_inertias[link_index]))
    }

    /// Returns the map from a force on link `k` in block `k`'s frame to its acceleration at joint
    /// `k + 1`'s origin in block `k + 1`'s frame.
    pub(super) fn shifted_parent_inverse_inertia(constants: &MassMatrixInverseConstants, link_index: usize, rotations: &[ZRotation]) -> Matrix6<f64> {
        rotations[link_index + 1].inverse().apply_rows(&constants.shifted_parent_inverse_inertias[link_index])
    }

    /// Returns the block coupling link `k`'s row to joint `k + 1`'s force.
    pub(super) fn child_coupling(constants: &MassMatrixInverseConstants, link_index: usize, rotations: &[ZRotation]) -> Matrix6<f64> {
        rotations[link_index + 1].apply_columns(&constants.child_couplings[link_index])
    }

    /// Returns `force`, transmitted across joint `k + 1` and given in block `k + 1`'s frame, at link
    /// `k`'s origin in block `k`'s frame.
    pub(super) fn child_force_in_parent(constants: &MassMatrixInverseConstants, link_index: usize, rotations: &[ZRotation], force: &Vector6<f64>) -> Vector6<f64> {
        let in_zero_angle_frame = rotations[link_index + 1].apply_spatial(force);
        let in_link = rotate_spatial(&constants.child_rotations[link_index], &in_zero_angle_frame);
        shift_force_back(&in_link, &constants.child_offsets[link_index])
    }

    /// Returns the component along joint `k + 1`'s axis of link `k`'s angular acceleration, given in
    /// block `k`'s frame.
    pub(super) fn parent_axis_acceleration(constants: &MassMatrixInverseConstants, link_index: usize, angular: &Vector3<f64>) -> f64 {
        constants.child_rotations[link_index].column(2).dot(angular)
    }
}

/// Configuration- and payload-dependent factors of the mass matrix inverse: the block-tridiagonal
/// matrix `𝒜` of the factorization `M⁻¹ = 𝒲ₐᵀ 𝒢 𝒜⁻¹ ℬ`, block-LU-factored, and the link terms
/// needed to recover joint accelerations from joint forces.
struct MassMatrixInverseFactorization<'a> {
    /// Configuration-independent terms of the chain.
    constants: &'a MassMatrixInverseConstants,
    /// Rotation of each joint by its angle.
    rotations: [ZRotation; NUM_JOINTS],
    /// Inertia terms of each link in its link frame.
    link_inertias: [AxisInertia; NUM_JOINTS],
    /// Block `𝒜[k + 1, k]` at index `k`.
    lower_blocks: [Matrix6<f64>; NUM_COUPLINGS],
    /// `D[k]⁻¹ 𝒜[k, k + 1]` at index `k`, for the block-LU pivots `D`.
    solved_upper_blocks: [Matrix6<f64>; NUM_COUPLINGS],
    /// LU decomposition of each block-LU pivot `D[i]`.
    pivots: [LU<f64, U6, U6>; NUM_JOINTS],
}

impl<'a> MassMatrixInverseFactorization<'a> {
    /// Assembles and factors `𝒜` at positions `q` with `payload` attached to the flange.
    fn new(chain: &KinematicChain, constants: &'a MassMatrixInverseConstants, payload: &RigidBodyInertia, q: &JointVector) -> Self {
        let rotations: [ZRotation; NUM_JOINTS] = std::array::from_fn(|joint_index| ZRotation::new(q[joint_index]));
        let last_link = AxisInertia::new(
            &chain.inertias_with_payload(payload)[NUM_JOINTS - 1].transformed(&constants.last_link_from_joint),
        );
        let link_inertias: [AxisInertia; NUM_JOINTS] = std::array::from_fn(|link_index| {
            // Every link but the last has constant terms; the last carries the payload.
            *constants.link_inertias.get(link_index).unwrap_or(&last_link)
        });
        let lower_blocks: [Matrix6<f64>; NUM_COUPLINGS] = std::array::from_fn(|link_index| {
            -link_inertias[link_index + 1]
                .remove_axis_component(&block_frame::shifted_parent_inverse_inertia(constants, link_index, &rotations))
        });
        let mut solved_upper_blocks = [Matrix6::zeros(); NUM_COUPLINGS];
        let pivots = std::array::from_fn(|block_index| {
            let mut pivot = link_inertias[block_index].inverse;
            if block_index > 0 {
                let parent_index = block_index - 1;
                pivot += link_inertias[block_index]
                    .remove_axis_component(&block_frame::parent_inverse_inertia(constants, parent_index, &rotations))
                    - lower_blocks[parent_index] * solved_upper_blocks[parent_index];
            }
            let decomposition = pivot.lu();
            if block_index < NUM_COUPLINGS {
                solved_upper_blocks[block_index] = decomposition
                    .solve(&block_frame::child_coupling(constants, block_index, &rotations))
                    .expect("block-LU pivot of a physical chain is invertible");
            }
            decomposition
        });
        Self {
            constants,
            rotations,
            link_inertias,
            lower_blocks,
            solved_upper_blocks,
            pivots,
        }
    }

    /// Returns column `joint_index` of the mass matrix inverse: the joint accelerations produced by
    /// a unit torque at that joint.
    fn column(&self, joint_index: usize) -> JointVector {
        // Joint forces solving 𝒜 F = ℬ τ. The right-hand side is zero outside block `joint_index`, so
        // the forward substitution is zero before it.
        let mut forces = [Vector6::zeros(); NUM_JOINTS];
        let mut right_hand_side = Vector6::zeros();
        right_hand_side[ANGULAR_Z] = self.link_inertias[joint_index].inverse_axis_inertia;
        forces[joint_index] = self.solve_pivot(joint_index, &right_hand_side);
        for block_index in joint_index + 1..NUM_JOINTS {
            forces[block_index] =
                self.solve_pivot(block_index, &(-(self.lower_blocks[block_index - 1] * forces[block_index - 1])));
        }
        for link_index in (0..NUM_COUPLINGS).rev() {
            forces[link_index] -= self.solved_upper_blocks[link_index] * forces[link_index + 1];
        }

        // Joint accelerations: each link's angular acceleration from its net force, then the joint's
        // share of the difference from its parent's.
        let mut accelerations = JointVector::zeros();
        let mut parent_angular = Vector3::zeros();
        for link_index in 0..NUM_JOINTS {
            let mut net_force = forces[link_index];
            if link_index < NUM_COUPLINGS {
                net_force -=
                    block_frame::child_force_in_parent(self.constants, link_index, &self.rotations, &forces[link_index + 1]);
            }
            let angular = self.link_inertias[link_index].inverse.fixed_rows::<3>(3) * net_force;
            accelerations[link_index] = angular.z;
            if link_index > 0 {
                accelerations[link_index] -=
                    block_frame::parent_axis_acceleration(self.constants, link_index - 1, &parent_angular);
            }
            parent_angular = angular;
        }
        accelerations
    }

    /// Returns `D[block_index]⁻¹ · vector`.
    fn solve_pivot(&self, block_index: usize, vector: &Vector6<f64>) -> Vector6<f64> {
        self.pivots[block_index]
            .solve(vector)
            .expect("block-LU pivot of a physical chain is invertible")
    }
}

/// Returns the inverse of the joint-space mass matrix at positions `q`, computed from the
/// factorization `M⁻¹ = 𝒲ₐᵀ 𝒢 𝒜⁻¹ ℬ` (Fijany and Featherstone, "A new factorization of the mass
/// matrix for optimal serial and parallel calculation of multibody dynamics", 2013).
///
/// `𝒜` is block tridiagonal and is block-LU-factored once. Each column then takes three steps for a
/// unit joint torque `τ`: the accelerations `ℬ τ` of the links disconnected from each other, the
/// joint forces `F` solving `𝒜 F = ℬ τ`, and the joint accelerations `𝒲ₐᵀ 𝒢 F` those forces produce.
///
/// `constants` must be [`MassMatrixInverseConstants::new`] of `chain`.
///
/// # Panics
/// Panics if the last link with `payload` has a singular rotational inertia, or if a block-LU pivot
/// of `𝒜` is singular; a chain of physical links produces neither.
pub fn mass_matrix_inverse(
    chain: &KinematicChain,
    constants: &MassMatrixInverseConstants,
    payload: &RigidBodyInertia,
    q: &JointVector,
) -> MassMatrix {
    let factorization = MassMatrixInverseFactorization::new(chain, constants, payload, q);
    let mut inverse = MassMatrix::zeros();
    for column in 0..NUM_JOINTS {
        inverse.set_column(column, &factorization.column(column));
    }
    inverse
}
