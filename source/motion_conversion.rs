//! Conversion of each motion type into the motion generator command sent to the robot, with the
//! input checks, low-pass filtering and rate limiting of libfranka's `ControlLoop::convertMotion`
//! (control loops) and `Robot::Impl::createMotionCommand` (active control).
//!
//! The trait here is the sealed supertrait of [`crate::control_types::MotionType`], so only the
//! motion types of this crate can be commanded.

use crate::constants::DELTA_T;
use crate::control_loop::{check_finite_array, ControlLoopConfig};
use crate::errors::{FrankaError, FrankaResult};
use crate::joint_velocity_limits::JointVelocityLimits;
use crate::lowpass_filter::{self, MAX_CUTOFF_FREQUENCY};
use crate::rate_limiting;
use crate::robot_state::RobotState;
use crate::types::{CartesianPose, CartesianVelocities, JointPositions, JointVelocities};
use crate::wire::robot::MotionGeneratorCommand;

/// Whether a control loop has produced its first motion command; the first command is filtered and
/// limited against itself instead of the robot's last commanded value.
pub struct FilterState {
    initialized: bool,
}

impl FilterState {
    /// Returns the state of a control loop that has not produced a command yet.
    pub fn new() -> Self {
        Self { initialized: false }
    }

    /// Returns true on the first call and false afterwards.
    fn first_command(&mut self) -> bool {
        !std::mem::replace(&mut self.initialized, true)
    }
}

/// Builds the motion generator command for one motion type.
pub trait ConvertMotion {
    /// Returns the command for a control-loop cycle: `self` low-pass filtered and rate limited as
    /// `config` selects, against `state` (or against itself on the first cycle).
    ///
    /// # Errors
    /// [`FrankaError::Realtime`] for a non-finite value and [`FrankaError::InvalidOperation`] for an
    /// invalid cutoff frequency, pose matrix or elbow, at the points where libfranka throws.
    fn control_loop_command(
        &self,
        state: &RobotState,
        config: &ControlLoopConfig,
        joint_velocity_limits: &JointVelocityLimits,
        filter_state: &mut FilterState,
    ) -> FrankaResult<MotionGeneratorCommand>;

    /// Returns the command for active control: `self` unchanged after libfranka's checks.
    ///
    /// # Errors
    /// As [`ConvertMotion::control_loop_command`].
    fn active_command(&self) -> FrankaResult<MotionGeneratorCommand>;
}

impl ConvertMotion for JointPositions {
    fn control_loop_command(
        &self,
        state: &RobotState,
        config: &ControlLoopConfig,
        joint_velocity_limits: &JointVelocityLimits,
        filter_state: &mut FilterState,
    ) -> FrankaResult<MotionGeneratorCommand> {
        let mut q_c = self.0;
        let reference = if filter_state.first_command() { q_c } else { state.q_d };
        if filters(config)? {
            check_filter_inputs(&q_c, &reference)?;
            q_c = lowpass_filter::lowpass_filter_joints(DELTA_T, &q_c, &reference, config.cutoff_frequency);
        }
        if config.limit_rate {
            check_finite_array(&q_c)?;
            q_c = rate_limiting::limit_rate_joint_positions(
                &joint_velocity_limits.upper(&reference),
                &joint_velocity_limits.lower(&reference),
                &rate_limiting::MAX_JOINT_ACCELERATION,
                &rate_limiting::MAX_JOINT_JERK,
                &q_c,
                &reference,
                &state.dq_d,
                &state.ddq_d,
            );
        }
        check_finite_array(&q_c)?;
        Ok(MotionGeneratorCommand { q_c, ..empty_command() })
    }

    fn active_command(&self) -> FrankaResult<MotionGeneratorCommand> {
        check_finite_array(&self.0)?;
        Ok(MotionGeneratorCommand { q_c: self.0, ..empty_command() })
    }
}

impl ConvertMotion for JointVelocities {
    fn control_loop_command(
        &self,
        state: &RobotState,
        config: &ControlLoopConfig,
        joint_velocity_limits: &JointVelocityLimits,
        _filter_state: &mut FilterState,
    ) -> FrankaResult<MotionGeneratorCommand> {
        let mut dq_c = self.0;
        if filters(config)? {
            check_filter_inputs(&dq_c, &state.dq_d)?;
            dq_c = lowpass_filter::lowpass_filter_joints(DELTA_T, &dq_c, &state.dq_d, config.cutoff_frequency);
        }
        if config.limit_rate {
            check_finite_array(&dq_c)?;
            dq_c = rate_limiting::limit_rate_joint_velocities(
                &joint_velocity_limits.upper(&state.q_d),
                &joint_velocity_limits.lower(&state.q_d),
                &rate_limiting::MAX_JOINT_ACCELERATION,
                &rate_limiting::MAX_JOINT_JERK,
                &dq_c,
                &state.dq_d,
                &state.ddq_d,
            );
        }
        check_finite_array(&dq_c)?;
        Ok(MotionGeneratorCommand { dq_c, ..empty_command() })
    }

    fn active_command(&self) -> FrankaResult<MotionGeneratorCommand> {
        check_finite_array(&self.0)?;
        Ok(MotionGeneratorCommand { dq_c: self.0, ..empty_command() })
    }
}

impl ConvertMotion for CartesianPose {
    fn control_loop_command(
        &self,
        state: &RobotState,
        config: &ControlLoopConfig,
        _joint_velocity_limits: &JointVelocityLimits,
        filter_state: &mut FilterState,
    ) -> FrankaResult<MotionGeneratorCommand> {
        let mut o_t_ee_c = self.to_column_major();
        // As libfranka: the first cycle's references are the command itself, later ones the
        // robot's last commanded pose and elbow.
        let (reference_pose, reference_elbow) = if filter_state.first_command() {
            (o_t_ee_c, self.elbow.unwrap_or_default())
        } else {
            (state.o_t_ee_c, state.elbow_c)
        };
        if filters(config)? {
            check_filter_inputs(&o_t_ee_c, &reference_pose)?;
            o_t_ee_c = lowpass_filter::cartesian_lowpass_filter(
                DELTA_T,
                &o_t_ee_c,
                &reference_pose,
                config.cutoff_frequency,
            );
        }
        if config.limit_rate {
            check_matrix(&o_t_ee_c)?;
            o_t_ee_c = rate_limiting::limit_rate_cartesian_pose(
                rate_limiting::MAX_TRANSLATIONAL_VELOCITY,
                rate_limiting::MAX_TRANSLATIONAL_ACCELERATION,
                rate_limiting::MAX_TRANSLATIONAL_JERK,
                rate_limiting::MAX_ROTATIONAL_VELOCITY,
                rate_limiting::MAX_ROTATIONAL_ACCELERATION,
                rate_limiting::MAX_ROTATIONAL_JERK,
                &o_t_ee_c,
                &reference_pose,
                &state.o_dp_ee_c,
                &state.o_ddp_ee_c,
            );
        }
        check_matrix(&o_t_ee_c)?;
        let (elbow_c, valid_elbow) = match self.elbow {
            Some(elbow) => (control_loop_elbow(elbow, reference_elbow[0], state, config)?, 1),
            None => ([0.0; 2], 0),
        };
        Ok(MotionGeneratorCommand { o_t_ee_c, elbow_c, valid_elbow, ..empty_command() })
    }

    fn active_command(&self) -> FrankaResult<MotionGeneratorCommand> {
        let o_t_ee_c = self.to_column_major();
        check_matrix(&o_t_ee_c)?;
        let (elbow_c, valid_elbow) = active_elbow(self.elbow)?;
        Ok(MotionGeneratorCommand { o_t_ee_c, elbow_c, valid_elbow, ..empty_command() })
    }
}

impl ConvertMotion for CartesianVelocities {
    fn control_loop_command(
        &self,
        state: &RobotState,
        config: &ControlLoopConfig,
        _joint_velocity_limits: &JointVelocityLimits,
        _filter_state: &mut FilterState,
    ) -> FrankaResult<MotionGeneratorCommand> {
        let mut o_dp_ee_c = self.to_array();
        if filters(config)? {
            check_filter_inputs(&o_dp_ee_c, &state.o_dp_ee_c)?;
            for (value, last) in o_dp_ee_c.iter_mut().zip(state.o_dp_ee_c) {
                *value = lowpass_filter::lowpass_filter(DELTA_T, *value, last, config.cutoff_frequency);
            }
        }
        if config.limit_rate {
            check_finite_array(&o_dp_ee_c)?;
            o_dp_ee_c = rate_limiting::limit_rate_cartesian_velocity(
                rate_limiting::MAX_TRANSLATIONAL_VELOCITY,
                rate_limiting::MAX_TRANSLATIONAL_ACCELERATION,
                rate_limiting::MAX_TRANSLATIONAL_JERK,
                rate_limiting::MAX_ROTATIONAL_VELOCITY,
                rate_limiting::MAX_ROTATIONAL_ACCELERATION,
                rate_limiting::MAX_ROTATIONAL_JERK,
                &o_dp_ee_c,
                &state.o_dp_ee_c,
                &state.o_ddp_ee_c,
            );
        }
        check_finite_array(&o_dp_ee_c)?;
        // As libfranka: the elbow reference is always the robot's last commanded elbow.
        let (elbow_c, valid_elbow) = match self.elbow {
            Some(elbow) => (control_loop_elbow(elbow, state.elbow_c[0], state, config)?, 1),
            None => ([0.0; 2], 0),
        };
        Ok(MotionGeneratorCommand { o_dp_ee_c, elbow_c, valid_elbow, ..empty_command() })
    }

    fn active_command(&self) -> FrankaResult<MotionGeneratorCommand> {
        let o_dp_ee_c = self.to_array();
        check_finite_array(&o_dp_ee_c)?;
        let (elbow_c, valid_elbow) = active_elbow(self.elbow)?;
        Ok(MotionGeneratorCommand { o_dp_ee_c, elbow_c, valid_elbow, ..empty_command() })
    }
}

/// Returns a motion generator command with every field zero (libfranka's value-initialized one).
pub fn empty_command() -> MotionGeneratorCommand {
    MotionGeneratorCommand {
        q_c: [0.0; 7],
        dq_c: [0.0; 7],
        o_t_ee_c: [0.0; 16],
        o_dp_ee_c: [0.0; 6],
        elbow_c: [0.0; 2],
        valid_elbow: 0,
        motion_generation_finished: 0,
    }
}

/// Returns whether `config` low-pass filters commands (cutoff below the maximum).
///
/// # Errors
/// [`FrankaError::InvalidOperation`] if the filter is on with a cutoff of zero or below, which
/// libfranka's filters reject.
pub fn filters(config: &ControlLoopConfig) -> FrankaResult<bool> {
    if config.cutoff_frequency >= MAX_CUTOFF_FREQUENCY || config.cutoff_frequency.is_nan() {
        return Ok(false);
    }
    if config.cutoff_frequency <= 0.0 {
        return Err(FrankaError::InvalidOperation {
            message: format!("cutoff frequency must be positive: {}", config.cutoff_frequency),
        });
    }
    Ok(true)
}

/// Returns an error unless the signal to filter and its last value are finite, as libfranka's
/// filters require.
fn check_filter_inputs<const N: usize>(current: &[f64; N], last: &[f64; N]) -> FrankaResult<()> {
    check_finite_array(current)?;
    if let Some(index) = last.iter().position(|value| !value.is_finite()) {
        return Err(FrankaError::Realtime {
            message: format!("element {index} of the robot's last commanded value is not finite: {}", last[index]),
        });
    }
    Ok(())
}

/// Returns an error unless `transform` is finite and a homogeneous transformation (column major),
/// as libfranka's `checkMatrix`: last row exactly (0, 0, 0, 1), and every rotation column and row
/// of unit length within 1e-5.
pub fn check_matrix(transform: &[f64; 16]) -> FrankaResult<()> {
    const ORTHONORMAL_THRESHOLD: f64 = 1e-5;
    check_finite_array(transform)?;
    let element = |row: usize, column: usize| transform[column * 4 + row];
    let unit = |a: f64, b: f64, c: f64| ((a.powi(2) + b.powi(2) + c.powi(2)).sqrt() - 1.0).abs() <= ORTHONORMAL_THRESHOLD;
    let homogeneous = element(3, 0) == 0.0
        && element(3, 1) == 0.0
        && element(3, 2) == 0.0
        && element(3, 3) == 1.0
        && (0..3).all(|column| unit(element(0, column), element(1, column), element(2, column)))
        && (0..3).all(|row| unit(element(row, 0), element(row, 1), element(row, 2)));
    if !homogeneous {
        return Err(FrankaError::InvalidOperation {
            message: "invalid transformation in motion generator: has to be a column-major homogeneous transformation".into(),
        });
    }
    Ok(())
}

/// Returns an error unless `elbow` is finite and its second element (the sign of joint 4) is
/// exactly +1 or -1, as libfranka's `checkElbow`.
pub fn check_elbow(elbow: &[f64; 2]) -> FrankaResult<()> {
    check_finite_array(elbow)?;
    if elbow[1] != 1.0 && elbow[1] != -1.0 {
        return Err(FrankaError::InvalidOperation {
            message: format!("invalid elbow: the sign of joint 4 must be +1 or -1, got {}", elbow[1]),
        });
    }
    Ok(())
}

/// Returns a control-loop elbow command: the elbow angle filtered and rate limited against
/// `reference` as libfranka does (position limiter with the robot's last elbow velocity and
/// acceleration), then checked.
fn control_loop_elbow(
    elbow: [f64; 2],
    reference: f64,
    state: &RobotState,
    config: &ControlLoopConfig,
) -> FrankaResult<[f64; 2]> {
    let mut elbow_c = elbow;
    if filters(config)? {
        check_filter_inputs(&[elbow_c[0]], &[reference])?;
        elbow_c[0] = lowpass_filter::lowpass_filter(DELTA_T, elbow_c[0], reference, config.cutoff_frequency);
    }
    if config.limit_rate {
        check_finite_array(&[elbow_c[0]])?;
        elbow_c[0] = rate_limiting::limit_rate_position(
            rate_limiting::MAX_ELBOW_VELOCITY,
            -rate_limiting::MAX_ELBOW_VELOCITY,
            rate_limiting::MAX_ELBOW_ACCELERATION,
            rate_limiting::MAX_ELBOW_JERK,
            elbow_c[0],
            reference,
            state.delbow_c[0],
            state.ddelbow_c[0],
        );
    }
    check_elbow(&elbow_c)?;
    Ok(elbow_c)
}

/// Returns the active-control elbow command and its valid flag: the elbow after libfranka's
/// check, or zeros and 0 without an elbow.
fn active_elbow(elbow: Option<[f64; 2]>) -> FrankaResult<([f64; 2], u8)> {
    match elbow {
        Some(elbow) => {
            check_elbow(&elbow)?;
            Ok((elbow, 1))
        }
        None => Ok(([0.0; 2], 0)),
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Isometry3;

    use super::*;
    use crate::wire::robot::RawRobotState;

    /// Returns a robot state decoded from an all-zero datagram.
    fn zero_state() -> RobotState {
        // SAFETY: the buffer is exactly one robot state long.
        unsafe { RawRobotState::from_bytes(&[0u8; RawRobotState::SIZE]) }.to_robot_state()
    }

    /// Returns a configuration with rate limiting on and the 100 Hz low-pass filter on or off.
    fn config(filter: bool) -> ControlLoopConfig {
        ControlLoopConfig {
            limit_rate: true,
            cutoff_frequency: if filter { lowpass_filter::DEFAULT_CUTOFF_FREQUENCY } else { MAX_CUTOFF_FREQUENCY },
        }
    }

    /// Returns `motion`'s control-loop command on its first cycle against an all-zero state.
    fn first_command<M: ConvertMotion>(motion: &M, filter: bool) -> FrankaResult<MotionGeneratorCommand> {
        motion.control_loop_command(&zero_state(), &config(filter), &JointVelocityLimits::default(), &mut FilterState::new())
    }

    #[test]
    fn joint_motions_reject_non_finite_values_with_limiting_on() {
        for filter in [false, true] {
            for bad_value in [f64::NAN, f64::INFINITY] {
                let mut values = [0.0; 7];
                values[2] = bad_value;
                let positions = first_command(&JointPositions::new(values), filter);
                let velocities = first_command(&JointVelocities::new(values), filter);
                assert!(matches!(positions, Err(FrankaError::Realtime { .. })), "{bad_value} filter {filter}: {positions:?}");
                assert!(matches!(velocities, Err(FrankaError::Realtime { .. })), "{bad_value} filter {filter}: {velocities:?}");
            }
        }
    }

    #[test]
    fn cartesian_velocities_reject_infinite_value_with_limiting_on() {
        let velocities = CartesianVelocities::from_array(&[f64::INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert!(matches!(first_command(&velocities, false), Err(FrankaError::Realtime { .. })));
    }

    #[test]
    fn elbow_sign_other_than_plus_or_minus_one_is_rejected() {
        let pose = CartesianPose::from_isometry(Isometry3::identity()).with_elbow([0.0, 0.5]);
        let velocities = CartesianVelocities::from_array(&[0.0; 6]).with_elbow([0.0, 0.0]);
        assert!(matches!(first_command(&pose, false), Err(FrankaError::InvalidOperation { .. })));
        assert!(matches!(velocities.active_command(), Err(FrankaError::InvalidOperation { .. })));
    }

    #[test]
    fn first_pose_cycle_limits_elbow_against_the_commanded_elbow() {
        // The state's elbow is 0; on the first cycle the reference is the command itself, so an
        // elbow angle of 1.0 passes unchanged instead of being limited toward 0.
        let pose = CartesianPose::from_isometry(Isometry3::identity()).with_elbow([1.0, -1.0]);
        let command = first_command(&pose, false).unwrap();
        assert_eq!({ command.elbow_c }, [1.0, -1.0]);
        assert_eq!(command.valid_elbow, 1);
    }

    #[test]
    fn cartesian_velocity_elbow_is_limited_against_the_robot_elbow() {
        let velocities = CartesianVelocities::from_array(&[0.0; 6]).with_elbow([1.0, 1.0]);
        let command = first_command(&velocities, false).unwrap();
        assert!(command.elbow_c[0] < 1.0e-3, "elbow {:?}", { command.elbow_c });
    }

    #[test]
    fn check_matrix_accepts_identity_and_rejects_non_homogeneous_matrices() {
        let mut matrix = [0.0; 16];
        for index in [0, 5, 10, 15] {
            matrix[index] = 1.0;
        }
        assert!(check_matrix(&matrix).is_ok());
        let mut scaled = matrix;
        scaled[0] = 1.001;
        assert!(matches!(check_matrix(&scaled), Err(FrankaError::InvalidOperation { .. })));
        let mut bottom_row = matrix;
        bottom_row[3] = 0.1;
        assert!(matches!(check_matrix(&bottom_row), Err(FrankaError::InvalidOperation { .. })));
    }

    #[test]
    fn non_positive_cutoff_is_rejected_and_nan_cutoff_disables_the_filter() {
        let config = |cutoff_frequency| ControlLoopConfig { limit_rate: false, cutoff_frequency };
        assert!(matches!(filters(&config(0.0)), Err(FrankaError::InvalidOperation { .. })));
        assert!(matches!(filters(&config(f64::NAN)), Ok(false)));
        assert!(matches!(filters(&config(100.0)), Ok(true)));
    }
}
