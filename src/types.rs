use std::ops::{Deref, DerefMut};

use nalgebra::{Isometry3, Matrix4, UnitQuaternion, Vector3};

use crate::constants::NUM_JOINTS;

/// Joint positions in radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointPositions(pub [f64; NUM_JOINTS]);

/// Joint velocities in rad/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointVelocities(pub [f64; NUM_JOINTS]);

/// Joint torques in Nm (without gravity and friction).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Torques(pub [f64; NUM_JOINTS]);

/// Cartesian pose as a homogeneous transformation (column-major 4x4 matrix).
///
/// Internally stored as `nalgebra::Isometry3<f64>` for proper SE(3) semantics,
/// but convertible to/from the column-major `[f64; 16]` wire format.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianPose {
    pub inner: Isometry3<f64>,
    pub elbow: Option<[f64; 2]>,
}

/// Cartesian velocities: linear (m/s) and angular (rad/s) components.
///
/// Expressed in the base frame with origin at the end effector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianVelocities {
    /// Linear velocity (x, y, z) in m/s.
    pub linear: Vector3<f64>,
    /// Angular velocity (wx, wy, wz) in rad/s.
    pub angular: Vector3<f64>,
    pub elbow: Option<[f64; 2]>,
}

/// Reference frame for kinematics computations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Frame {
    Joint1,
    Joint2,
    Joint3,
    Joint4,
    Joint5,
    Joint6,
    Joint7,
    Flange,
    EndEffector,
    Stiffness,
}

/// Robot operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotMode {
    Other,
    Idle,
    Move,
    Guiding,
    Reflex,
    UserStopped,
    AutomaticErrorRecovery,
}

/// Active controller mode on the robot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerMode {
    JointImpedance,
    CartesianImpedance,
    ExternalController,
}

/// Whether to enforce real-time scheduling for the control loop thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RealtimeConfig {
    Enforce,
    Ignore,
}

/// Motion generator mode (internal, reflects what the robot is currently doing).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionGeneratorMode {
    Idle,
    JointPosition,
    JointVelocity,
    CartesianPosition,
    CartesianVelocity,
    None,
}

// === Newtype impls ===

impl JointPositions {
    pub fn new(values: [f64; NUM_JOINTS]) -> Self {
        Self(values)
    }
}

impl JointVelocities {
    pub fn new(values: [f64; NUM_JOINTS]) -> Self {
        Self(values)
    }
}

impl Torques {
    pub fn new(values: [f64; NUM_JOINTS]) -> Self {
        Self(values)
    }
}

impl CartesianPose {
    pub fn from_isometry(isometry: Isometry3<f64>) -> Self {
        Self {
            inner: isometry,
            elbow: None,
        }
    }

    pub fn with_elbow(mut self, elbow: [f64; 2]) -> Self {
        self.elbow = Some(elbow);
        self
    }

    /// Create from a column-major 4x4 homogeneous transformation matrix.
    pub fn from_column_major(data: &[f64; 16]) -> Self {
        let mat = Matrix4::from_column_slice(data);
        let isometry = Isometry3::from_parts(
            Vector3::new(mat[(0, 3)], mat[(1, 3)], mat[(2, 3)]).into(),
            UnitQuaternion::from_matrix(&mat.fixed_view::<3, 3>(0, 0).into()),
        );
        Self {
            inner: isometry,
            elbow: None,
        }
    }

    /// Convert to a column-major 4x4 homogeneous transformation matrix.
    pub fn to_column_major(&self) -> [f64; 16] {
        let mat = self.inner.to_homogeneous();
        let mut out = [0.0; 16];
        out.copy_from_slice(mat.as_slice());
        out
    }
}

impl CartesianVelocities {
    pub fn new(linear: Vector3<f64>, angular: Vector3<f64>) -> Self {
        Self {
            linear,
            angular,
            elbow: None,
        }
    }

    pub fn from_array(data: &[f64; 6]) -> Self {
        Self {
            linear: Vector3::new(data[0], data[1], data[2]),
            angular: Vector3::new(data[3], data[4], data[5]),
            elbow: None,
        }
    }

    pub fn with_elbow(mut self, elbow: [f64; 2]) -> Self {
        self.elbow = Some(elbow);
        self
    }

    pub fn to_array(&self) -> [f64; 6] {
        [
            self.linear.x,
            self.linear.y,
            self.linear.z,
            self.angular.x,
            self.angular.y,
            self.angular.z,
        ]
    }
}

// Deref impls for joint types to allow easy access to the underlying array.

impl Deref for JointPositions {
    type Target = [f64; NUM_JOINTS];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for JointPositions {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Deref for JointVelocities {
    type Target = [f64; NUM_JOINTS];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for JointVelocities {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Deref for Torques {
    type Target = [f64; NUM_JOINTS];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Torques {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl From<[f64; NUM_JOINTS]> for JointPositions {
    fn from(values: [f64; NUM_JOINTS]) -> Self {
        Self(values)
    }
}

impl From<[f64; NUM_JOINTS]> for JointVelocities {
    fn from(values: [f64; NUM_JOINTS]) -> Self {
        Self(values)
    }
}

impl From<[f64; NUM_JOINTS]> for Torques {
    fn from(values: [f64; NUM_JOINTS]) -> Self {
        Self(values)
    }
}

impl RobotMode {
    pub(crate) fn from_wire(value: u8) -> Self {
        match value {
            1 => Self::Idle,
            2 => Self::Move,
            3 => Self::Guiding,
            4 => Self::Reflex,
            5 => Self::UserStopped,
            6 => Self::AutomaticErrorRecovery,
            _ => Self::Other,
        }
    }
}

impl MotionGeneratorMode {
    pub(crate) fn from_wire(value: u8) -> Self {
        match value {
            0 => Self::Idle,
            1 => Self::JointPosition,
            2 => Self::JointVelocity,
            3 => Self::CartesianPosition,
            4 => Self::CartesianVelocity,
            _ => Self::None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn joint_positions_deref() {
        let jp = JointPositions::new([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
        assert_eq!(jp[0], 1.0);
        assert_eq!(jp[6], 7.0);
        assert_eq!(jp.len(), 7);
    }

    #[test]
    fn joint_positions_from_array() {
        let arr = [0.1; 7];
        let jp: JointPositions = arr.into();
        assert_eq!(jp.0, arr);
    }

    #[test]
    fn cartesian_pose_roundtrip() {
        let identity = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let pose = CartesianPose::from_column_major(&identity);
        let back = pose.to_column_major();
        for (a, b) in identity.iter().zip(back.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn cartesian_pose_translation() {
        let mat = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.5, 0.3, 0.1, 1.0,
        ];
        let pose = CartesianPose::from_column_major(&mat);
        let t = pose.inner.translation;
        assert!((t.x - 0.5).abs() < 1e-10);
        assert!((t.y - 0.3).abs() < 1e-10);
        assert!((t.z - 0.1).abs() < 1e-10);
    }

    #[test]
    fn cartesian_velocities_array_roundtrip() {
        let arr = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3];
        let cv = CartesianVelocities::from_array(&arr);
        let back = cv.to_array();
        assert_eq!(arr, back);
    }

    #[test]
    fn robot_mode_from_wire() {
        assert_eq!(RobotMode::from_wire(0), RobotMode::Other);
        assert_eq!(RobotMode::from_wire(1), RobotMode::Idle);
        assert_eq!(RobotMode::from_wire(2), RobotMode::Move);
        assert_eq!(RobotMode::from_wire(5), RobotMode::UserStopped);
        assert_eq!(RobotMode::from_wire(255), RobotMode::Other);
    }
}
