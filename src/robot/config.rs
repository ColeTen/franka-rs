use crate::control_loop::ControlLoopConfig;
use crate::lowpass_filter::DEFAULT_CUTOFF_FREQUENCY;
use crate::types::ControllerMode;
use crate::wire::robot::{SetCollisionBehaviorRequest, SetLoadRequest};

/// Builder for collision behavior thresholds.
///
/// Forces/torques between lower and upper thresholds are shown as contacts.
/// Forces/torques above upper thresholds are registered as collisions (robot stops).
#[derive(Debug, Clone)]
pub struct CollisionConfig {
    pub lower_torque_thresholds_acceleration: [f64; 7],
    pub upper_torque_thresholds_acceleration: [f64; 7],
    pub lower_torque_thresholds_nominal: [f64; 7],
    pub upper_torque_thresholds_nominal: [f64; 7],
    pub lower_force_thresholds_acceleration: [f64; 6],
    pub upper_force_thresholds_acceleration: [f64; 6],
    pub lower_force_thresholds_nominal: [f64; 6],
    pub upper_force_thresholds_nominal: [f64; 6],
}

impl CollisionConfig {
    /// Create with symmetric thresholds (same for acceleration and nominal phases).
    pub fn symmetric(
        lower_torque: [f64; 7],
        upper_torque: [f64; 7],
        lower_force: [f64; 6],
        upper_force: [f64; 6],
    ) -> Self {
        Self {
            lower_torque_thresholds_acceleration: lower_torque,
            upper_torque_thresholds_acceleration: upper_torque,
            lower_torque_thresholds_nominal: lower_torque,
            upper_torque_thresholds_nominal: upper_torque,
            lower_force_thresholds_acceleration: lower_force,
            upper_force_thresholds_acceleration: upper_force,
            lower_force_thresholds_nominal: lower_force,
            upper_force_thresholds_nominal: upper_force,
        }
    }

    pub(crate) fn to_request(&self) -> SetCollisionBehaviorRequest {
        SetCollisionBehaviorRequest {
            lower_torque_thresholds_acceleration: self.lower_torque_thresholds_acceleration,
            upper_torque_thresholds_acceleration: self.upper_torque_thresholds_acceleration,
            lower_torque_thresholds_nominal: self.lower_torque_thresholds_nominal,
            upper_torque_thresholds_nominal: self.upper_torque_thresholds_nominal,
            lower_force_thresholds_acceleration: self.lower_force_thresholds_acceleration,
            upper_force_thresholds_acceleration: self.upper_force_thresholds_acceleration,
            lower_force_thresholds_nominal: self.lower_force_thresholds_nominal,
            upper_force_thresholds_nominal: self.upper_force_thresholds_nominal,
        }
    }
}

/// Payload (external load) parameters.
#[derive(Debug, Clone)]
pub struct LoadConfig {
    pub mass: f64,
    pub center_of_mass: [f64; 3],
    pub inertia: [f64; 9],
}

impl LoadConfig {
    pub fn new(mass: f64, center_of_mass: [f64; 3], inertia: [f64; 9]) -> Self {
        Self {
            mass,
            center_of_mass,
            inertia,
        }
    }

    pub(crate) fn to_request(&self) -> SetLoadRequest {
        SetLoadRequest {
            m_load: self.mass,
            f_x_cload: self.center_of_mass,
            i_load: self.inertia,
        }
    }
}

/// Configuration for motion control methods.
#[derive(Debug, Clone)]
pub struct MotionConfig {
    pub controller_mode: ControllerMode,
    pub limit_rate: bool,
    pub cutoff_frequency: f64,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            controller_mode: ControllerMode::JointImpedance,
            limit_rate: true,
            cutoff_frequency: DEFAULT_CUTOFF_FREQUENCY,
        }
    }
}

impl MotionConfig {
    pub fn with_controller_mode(mut self, mode: ControllerMode) -> Self {
        self.controller_mode = mode;
        self
    }

    pub fn with_rate_limiting(mut self, enabled: bool) -> Self {
        self.limit_rate = enabled;
        self
    }

    pub fn with_cutoff_frequency(mut self, frequency: f64) -> Self {
        self.cutoff_frequency = frequency;
        self
    }

    pub(crate) fn to_control_loop_config(&self) -> ControlLoopConfig {
        ControlLoopConfig {
            limit_rate: self.limit_rate,
            cutoff_frequency: self.cutoff_frequency,
        }
    }
}
