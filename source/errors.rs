use bitflags::bitflags;
use thiserror::Error;

use crate::robot_state::RobotState;

bitflags! {
    /// Robot error flags. Each flag corresponds to a specific safety or limit violation.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct RobotErrors: u64 {
        const JOINT_POSITION_LIMITS_VIOLATION = 1 << 0;
        const CARTESIAN_POSITION_LIMITS_VIOLATION = 1 << 1;
        const SELF_COLLISION_AVOIDANCE_VIOLATION = 1 << 2;
        const JOINT_VELOCITY_VIOLATION = 1 << 3;
        const CARTESIAN_VELOCITY_VIOLATION = 1 << 4;
        const FORCE_CONTROL_SAFETY_VIOLATION = 1 << 5;
        const JOINT_REFLEX = 1 << 6;
        const CARTESIAN_REFLEX = 1 << 7;
        const MAX_GOAL_POSE_DEVIATION_VIOLATION = 1 << 8;
        const MAX_PATH_POSE_DEVIATION_VIOLATION = 1 << 9;
        const CARTESIAN_VELOCITY_PROFILE_SAFETY_VIOLATION = 1 << 10;
        const JOINT_POSITION_MOTION_GENERATOR_START_POSE_INVALID = 1 << 11;
        const JOINT_MOTION_GENERATOR_POSITION_LIMITS_VIOLATION = 1 << 12;
        const JOINT_MOTION_GENERATOR_VELOCITY_LIMITS_VIOLATION = 1 << 13;
        const JOINT_MOTION_GENERATOR_VELOCITY_DISCONTINUITY = 1 << 14;
        const JOINT_MOTION_GENERATOR_ACCELERATION_DISCONTINUITY = 1 << 15;
        const CARTESIAN_POSITION_MOTION_GENERATOR_START_POSE_INVALID = 1 << 16;
        const CARTESIAN_MOTION_GENERATOR_ELBOW_LIMIT_VIOLATION = 1 << 17;
        const CARTESIAN_MOTION_GENERATOR_VELOCITY_LIMITS_VIOLATION = 1 << 18;
        const CARTESIAN_MOTION_GENERATOR_VELOCITY_DISCONTINUITY = 1 << 19;
        const CARTESIAN_MOTION_GENERATOR_ACCELERATION_DISCONTINUITY = 1 << 20;
        const CARTESIAN_MOTION_GENERATOR_ELBOW_SIGN_INCONSISTENT = 1 << 21;
        const CARTESIAN_MOTION_GENERATOR_START_ELBOW_INVALID = 1 << 22;
        const CARTESIAN_MOTION_GENERATOR_JOINT_POSITION_LIMITS_VIOLATION = 1 << 23;
        const CARTESIAN_MOTION_GENERATOR_JOINT_VELOCITY_LIMITS_VIOLATION = 1 << 24;
        const CARTESIAN_MOTION_GENERATOR_JOINT_VELOCITY_DISCONTINUITY = 1 << 25;
        const CARTESIAN_MOTION_GENERATOR_JOINT_ACCELERATION_DISCONTINUITY = 1 << 26;
        const CARTESIAN_POSITION_MOTION_GENERATOR_INVALID_FRAME = 1 << 27;
        const FORCE_CONTROLLER_DESIRED_FORCE_TOLERANCE_VIOLATION = 1 << 28;
        const CONTROLLER_TORQUE_DISCONTINUITY = 1 << 29;
        const START_ELBOW_SIGN_INCONSISTENT = 1 << 30;
        const COMMUNICATION_CONSTRAINTS_VIOLATION = 1 << 31;
        const POWER_LIMIT_VIOLATION = 1 << 32;
        const JOINT_P2P_INSUFFICIENT_TORQUE_FOR_PLANNING = 1 << 33;
        const TAU_J_RANGE_VIOLATION = 1 << 34;
        const INSTABILITY_DETECTED = 1 << 35;
        const JOINT_MOVE_IN_WRONG_DIRECTION = 1 << 36;
        const CARTESIAN_SPLINE_MOTION_GENERATOR_VIOLATION = 1 << 37;
        const JOINT_VIA_MOTION_GENERATOR_PLANNING_JOINT_LIMIT_VIOLATION = 1 << 38;
        const BASE_ACCELERATION_INITIALIZATION_TIMEOUT = 1 << 39;
        const BASE_ACCELERATION_INVALID_READING = 1 << 40;
    }
}

impl RobotErrors {
    /// Construct from the wire-format boolean array.
    pub fn from_bool_array(errors: &[bool; 41]) -> Self {
        let mut flags = Self::empty();
        for (i, &active) in errors.iter().enumerate() {
            if active {
                flags |= Self::from_bits_truncate(1 << i);
            }
        }
        flags
    }

    /// Returns true if any errors are active.
    pub fn has_errors(&self) -> bool {
        !self.is_empty()
    }
}

/// Errors that can occur when communicating with or controlling the robot.
#[derive(Debug, Error)]
pub enum FrankaError {
    #[error("network error: {message}")]
    Network {
        message: String,
        #[source]
        source: Option<std::io::Error>,
    },

    #[error("protocol error: {message}")]
    Protocol { message: String },

    #[error("incompatible version: server={server_version}, library={library_version}")]
    IncompatibleVersion {
        server_version: u16,
        library_version: u16,
    },

    #[error("control error: {message}")]
    Control {
        message: String,
        log: Vec<RobotState>,
    },

    #[error("command rejected: {message}")]
    Command { message: String },

    #[error("realtime error: {message}")]
    Realtime { message: String },

    #[error("model error: {message}")]
    Model { message: String },

    #[error("invalid operation: {message}")]
    InvalidOperation { message: String },
}

/// Convenience type alias for results from franka-rs operations.
pub type FrankaResult<T> = Result<T, FrankaError>;

impl FrankaError {
    pub fn network(message: impl Into<String>) -> Self {
        Self::Network {
            message: message.into(),
            source: None,
        }
    }

    pub fn network_with_source(message: impl Into<String>, source: std::io::Error) -> Self {
        Self::Network {
            message: message.into(),
            source: Some(source),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robot_errors_from_bool_array_empty() {
        let bools = [false; 41];
        let errors = RobotErrors::from_bool_array(&bools);
        assert!(!errors.has_errors());
        assert!(errors.is_empty());
    }

    #[test]
    fn robot_errors_from_bool_array_single() {
        let mut bools = [false; 41];
        bools[0] = true;
        let errors = RobotErrors::from_bool_array(&bools);
        assert!(errors.has_errors());
        assert!(errors.contains(RobotErrors::JOINT_POSITION_LIMITS_VIOLATION));
        assert!(!errors.contains(RobotErrors::CARTESIAN_REFLEX));
    }

    #[test]
    fn robot_errors_from_bool_array_multiple() {
        let mut bools = [false; 41];
        bools[6] = true; // JOINT_REFLEX
        bools[7] = true; // CARTESIAN_REFLEX
        bools[40] = true; // BASE_ACCELERATION_INVALID_READING
        let errors = RobotErrors::from_bool_array(&bools);
        assert!(errors.contains(RobotErrors::JOINT_REFLEX));
        assert!(errors.contains(RobotErrors::CARTESIAN_REFLEX));
        assert!(errors.contains(RobotErrors::BASE_ACCELERATION_INVALID_READING));
        assert!(!errors.contains(RobotErrors::POWER_LIMIT_VIOLATION));
    }

    #[test]
    fn franka_error_display() {
        let err = FrankaError::network("connection timeout");
        assert_eq!(err.to_string(), "network error: connection timeout");

        let err = FrankaError::IncompatibleVersion {
            server_version: 10,
            library_version: 9,
        };
        assert_eq!(
            err.to_string(),
            "incompatible version: server=10, library=9"
        );
    }
}
