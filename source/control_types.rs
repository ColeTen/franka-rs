use std::ops::ControlFlow;

use crate::types::{CartesianPose, CartesianVelocities, JointPositions, JointVelocities};

/// Motion command returned by a user's control callback.
///
/// Users return `ControlFlow::Continue(cmd)` to keep the motion going,
/// or `ControlFlow::Break(cmd)` to send the final command and stop.
pub type MotionResult<T> = ControlFlow<T, T>;

/// Trait for types that can be used as motion commands: joint positions, joint velocities,
/// Cartesian pose and Cartesian velocities. It is sealed; how each type becomes a robot command is
/// defined in this crate.
pub trait MotionType: Clone + Copy + std::fmt::Debug + crate::motion_conversion::ConvertMotion {
    /// The wire-format motion generator mode for this type.
    fn motion_generator_mode() -> crate::types::MotionGeneratorMode;
}

impl MotionType for JointPositions {
    fn motion_generator_mode() -> crate::types::MotionGeneratorMode {
        crate::types::MotionGeneratorMode::JointPosition
    }
}

impl MotionType for JointVelocities {
    fn motion_generator_mode() -> crate::types::MotionGeneratorMode {
        crate::types::MotionGeneratorMode::JointVelocity
    }
}

impl MotionType for CartesianPose {
    fn motion_generator_mode() -> crate::types::MotionGeneratorMode {
        crate::types::MotionGeneratorMode::CartesianPosition
    }
}

impl MotionType for CartesianVelocities {
    fn motion_generator_mode() -> crate::types::MotionGeneratorMode {
        crate::types::MotionGeneratorMode::CartesianVelocity
    }
}

/// Extracts the inner value from a `MotionResult`, regardless of whether it's Continue or Break.
pub fn motion_value<T: Copy>(result: &MotionResult<T>) -> T {
    match result {
        ControlFlow::Continue(v) | ControlFlow::Break(v) => *v,
    }
}

/// Returns true if the motion result signals completion.
pub fn is_finished<T>(result: &MotionResult<T>) -> bool {
    matches!(result, ControlFlow::Break(_))
}
