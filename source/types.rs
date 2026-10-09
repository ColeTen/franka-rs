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

/// Cartesian pose command: the end-effector pose in the base frame and an optional elbow.
///
/// The pose is stored exactly as given, as a column-major 4x4 homogeneous transformation (the
/// robot's format), and sent unchanged when filtering and rate limiting are off. It is checked to
/// be a homogeneous transformation when it becomes a command, as libfranka does. Use
/// [`CartesianPose::from_isometry`] and [`CartesianPose::to_isometry`] for pose arithmetic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianPose {
    /// End-effector pose in the base frame, column-major 4x4 homogeneous transformation.
    pub o_t_ee: [f64; 16],
    /// Elbow configuration: the joint-3 angle (rad) and the sign of joint 4 (+1 or -1).
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

/// Robot operating mode, numbered as in the robot state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RobotMode {
    Other = 0,
    Idle = 1,
    Move = 2,
    Guiding = 3,
    Reflex = 4,
    UserStopped = 5,
    AutomaticErrorRecovery = 6,
}

/// Active controller mode on the robot, numbered as in the robot state (which additionally uses 3
/// for "other", see `wire::robot::CONTROLLER_MODE_OTHER`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControllerMode {
    JointImpedance = 0,
    CartesianImpedance = 1,
    ExternalController = 2,
}

/// Whether to enforce real-time scheduling for the control loop thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RealtimeConfig {
    Enforce,
    Ignore,
}

/// Motion generator mode (reflects what the robot is currently doing), numbered as in the robot
/// state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MotionGeneratorMode {
    Idle = 0,
    JointPosition = 1,
    JointVelocity = 2,
    CartesianPosition = 3,
    CartesianVelocity = 4,
    None = 5,
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
    /// Returns the pose of `isometry`, without an elbow.
    pub fn from_isometry(isometry: Isometry3<f64>) -> Self {
        let mut o_t_ee = [0.0; 16];
        o_t_ee.copy_from_slice(isometry.to_homogeneous().as_slice());
        Self { o_t_ee, elbow: None }
    }

    pub fn with_elbow(mut self, elbow: [f64; 2]) -> Self {
        self.elbow = Some(elbow);
        self
    }

    /// Returns the pose of a column-major 4x4 homogeneous transformation, stored exactly as given,
    /// without an elbow.
    pub fn from_column_major(data: &[f64; 16]) -> Self {
        Self { o_t_ee: *data, elbow: None }
    }

    /// Returns the pose as a column-major 4x4 homogeneous transformation, exactly as stored.
    pub fn to_column_major(&self) -> [f64; 16] {
        self.o_t_ee
    }

    /// Returns the pose as an isometry.
    ///
    /// # Errors
    /// [`crate::errors::FrankaError::InvalidArgument`] if the stored matrix is not finite or not a
    /// homogeneous transformation; an invalid matrix is reported, never repaired.
    pub fn to_isometry(&self) -> crate::errors::FrankaResult<Isometry3<f64>> {
        crate::command_checks::check_matrix(&self.o_t_ee)?;
        Ok(isometry_from_column_major(&self.o_t_ee))
    }
}

/// Returns the isometry of a column-major 4x4 transformation: its translation, and the unit
/// quaternion nearest to its rotation block.
pub(crate) fn isometry_from_column_major(data: &[f64; 16]) -> Isometry3<f64> {
    let mat = Matrix4::from_column_slice(data);
    Isometry3::from_parts(
        Vector3::new(mat[(0, 3)], mat[(1, 3)], mat[(2, 3)]).into(),
        UnitQuaternion::from_matrix(&mat.fixed_view::<3, 3>(0, 0).into()),
    )
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
        assert_eq!(pose.to_column_major(), identity);
    }

    #[test]
    fn cartesian_pose_keeps_matrix_bits_and_reports_invalid_matrix() {
        // A rotation about z by 0.3 rad, slightly off orthonormal in the last bits: stored and
        // returned exactly, while a matrix with a scaled column cannot become an isometry.
        let (c, s) = (0.3_f64.cos(), 0.3_f64.sin());
        let matrix = [c, s, 0.0, 0.0, -s, c, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.2, 0.3, 1.0];
        assert_eq!(CartesianPose::from_column_major(&matrix).to_column_major(), matrix);
        let mut scaled = matrix;
        scaled[0] *= 1.01;
        assert!(matches!(
            CartesianPose::from_column_major(&scaled).to_isometry(),
            Err(crate::errors::FrankaError::InvalidArgument { .. })
        ));
    }

    #[test]
    fn cartesian_pose_translation() {
        let mat = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.5, 0.3, 0.1, 1.0,
        ];
        let pose = CartesianPose::from_column_major(&mat);
        let t = pose.to_isometry().unwrap().translation;
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
