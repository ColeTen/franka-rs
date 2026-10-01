//! Forward kinematics and Jacobians of a [`KinematicChain`].

use nalgebra::Isometry3;

use super::chain::KinematicChain;
use super::spatial::SpatialMotion;
use super::{Jacobian, JointVector};
use crate::constants::NUM_JOINTS;
use crate::types::Frame;

/// Returns the pose of every joint frame in the base frame at joint positions `q`.
pub fn joint_poses(chain: &KinematicChain, q: &JointVector) -> [Isometry3<f64>; NUM_JOINTS] {
    let mut poses = [Isometry3::identity(); NUM_JOINTS];
    let mut base_from_joint = Isometry3::identity();
    for (joint_index, pose) in poses.iter_mut().enumerate() {
        base_from_joint *= chain.parent_from_joint(joint_index, q[joint_index]);
        *pose = base_from_joint;
    }
    poses
}

/// Returns the index of the joint that `frame` moves with and the frame's pose in that joint's frame.
///
/// `f_t_ee` is the end-effector pose in the flange frame and `ee_t_k` the stiffness pose in the
/// end-effector frame.
fn frame_placement(
    chain: &KinematicChain,
    frame: Frame,
    f_t_ee: &Isometry3<f64>,
    ee_t_k: &Isometry3<f64>,
) -> (usize, Isometry3<f64>) {
    let last = NUM_JOINTS - 1;
    match frame {
        Frame::Joint1 => (0, Isometry3::identity()),
        Frame::Joint2 => (1, Isometry3::identity()),
        Frame::Joint3 => (2, Isometry3::identity()),
        Frame::Joint4 => (3, Isometry3::identity()),
        Frame::Joint5 => (4, Isometry3::identity()),
        Frame::Joint6 => (5, Isometry3::identity()),
        Frame::Joint7 => (last, Isometry3::identity()),
        Frame::Flange => (last, chain.flange),
        Frame::EndEffector => (last, chain.flange * f_t_ee),
        Frame::Stiffness => (last, chain.flange * f_t_ee * ee_t_k),
    }
}

/// Returns the pose of `frame` in the base frame at joint positions `q`.
pub fn frame_pose(
    chain: &KinematicChain,
    frame: Frame,
    q: &JointVector,
    f_t_ee: &Isometry3<f64>,
    ee_t_k: &Isometry3<f64>,
) -> Isometry3<f64> {
    let (joint_index, placement) = frame_placement(chain, frame, f_t_ee, ee_t_k);
    joint_poses(chain, q)[joint_index] * placement
}

/// Returns the base-frame unit twist of each joint, zero for joints that do not move `frame`,
/// together with the pose of `frame` in the base frame.
fn world_joint_twists(
    chain: &KinematicChain,
    frame: Frame,
    q: &JointVector,
    f_t_ee: &Isometry3<f64>,
    ee_t_k: &Isometry3<f64>,
) -> ([SpatialMotion; NUM_JOINTS], Isometry3<f64>) {
    let (frame_joint_index, placement) = frame_placement(chain, frame, f_t_ee, ee_t_k);
    let poses = joint_poses(chain, q);
    let mut twists = [SpatialMotion::zero(); NUM_JOINTS];
    for (joint_index, twist) in twists.iter_mut().enumerate().take(frame_joint_index + 1) {
        *twist = SpatialMotion::revolute(&chain.joints[joint_index].axis, 1.0).act(&poses[joint_index]);
    }
    (twists, poses[frame_joint_index] * placement)
}

/// Writes `twist` as column `column` of `jacobian`, linear rows first.
fn set_column(jacobian: &mut Jacobian, column: usize, twist: &SpatialMotion) {
    jacobian.fixed_view_mut::<3, 1>(0, column).copy_from(&twist.linear);
    jacobian.fixed_view_mut::<3, 1>(3, column).copy_from(&twist.angular);
}

/// Returns the 6×7 Jacobian mapping joint velocities to the velocity of `frame`'s origin and the
/// frame's angular velocity, both expressed in base-frame axes.
pub fn zero_jacobian(
    chain: &KinematicChain,
    frame: Frame,
    q: &JointVector,
    f_t_ee: &Isometry3<f64>,
    ee_t_k: &Isometry3<f64>,
) -> Jacobian {
    let (twists, base_from_frame) = world_joint_twists(chain, frame, q, f_t_ee, ee_t_k);
    let frame_origin = base_from_frame.translation.vector;
    let mut jacobian = Jacobian::zeros();
    for (column, twist) in twists.iter().enumerate() {
        let twist_at_frame_origin = SpatialMotion {
            linear: twist.linear - frame_origin.cross(&twist.angular),
            angular: twist.angular,
        };
        set_column(&mut jacobian, column, &twist_at_frame_origin);
    }
    jacobian
}

/// Returns the 6×7 Jacobian mapping joint velocities to the twist of `frame` expressed in
/// `frame` itself.
pub fn body_jacobian(
    chain: &KinematicChain,
    frame: Frame,
    q: &JointVector,
    f_t_ee: &Isometry3<f64>,
    ee_t_k: &Isometry3<f64>,
) -> Jacobian {
    let (twists, base_from_frame) = world_joint_twists(chain, frame, q, f_t_ee, ee_t_k);
    let mut jacobian = Jacobian::zeros();
    for (column, twist) in twists.iter().enumerate() {
        set_column(&mut jacobian, column, &twist.act_inverse(&base_from_frame));
    }
    jacobian
}
