//! Geometric and inertial description of the arm as a serial chain of revolute joints.

use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

use super::spatial::RigidBodyInertia;
use crate::constants::NUM_JOINTS;

/// One revolute joint together with the link it drives.
#[derive(Debug, Clone, PartialEq)]
pub struct RevoluteJoint {
    /// Pose of this joint's frame at zero angle, expressed in the previous joint's frame
    /// (the base frame for the first joint).
    pub placement: Isometry3<f64>,
    /// Rotation axis, expressed in this joint's frame.
    pub axis: Unit<Vector3<f64>>,
    /// Inertia of every body rigidly attached to this joint, expressed in this joint's frame.
    pub inertia: RigidBodyInertia,
}

/// Serial chain of the arm's joints from base to flange.
#[derive(Debug, Clone, PartialEq)]
pub struct KinematicChain {
    /// Joints ordered from base to tip.
    pub joints: [RevoluteJoint; NUM_JOINTS],
    /// Pose of the flange frame, expressed in the last joint's frame.
    pub flange: Isometry3<f64>,
}

impl KinematicChain {
    /// Returns the pose of joint `joint_index`'s frame in the previous joint's frame at `angle`.
    pub(crate) fn parent_from_joint(&self, joint_index: usize, angle: f64) -> Isometry3<f64> {
        let joint = &self.joints[joint_index];
        joint.placement
            * Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_axis_angle(&joint.axis, angle),
            )
    }

    /// Returns the joint inertias with `payload`, given in the flange frame, rigidly attached
    /// to the last joint.
    pub(crate) fn inertias_with_payload(
        &self,
        payload: &RigidBodyInertia,
    ) -> [RigidBodyInertia; NUM_JOINTS] {
        let mut inertias = self.joints.each_ref().map(|joint| joint.inertia);
        let last = NUM_JOINTS - 1;
        inertias[last] = inertias[last].combine(&payload.transformed(&self.flange));
        inertias
    }
}
