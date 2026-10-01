use std::time::Duration;

use crate::constants::DELTA_T;
use crate::control_types::{is_finished, motion_value, MotionResult, MotionType};
use crate::errors::{FrankaError, FrankaResult};
use crate::logging::Logger;
use crate::lowpass_filter::{self, MAX_CUTOFF_FREQUENCY};
use crate::network::Network;
use crate::rate_limiting;
use crate::robot_state::RobotState;
use crate::types::{
    CartesianPose, CartesianVelocities, ControllerMode, JointPositions, JointVelocities,
    MotionGeneratorMode, Torques,
};
use crate::wire::robot::{
    self, ControllerCommand, MotionGeneratorCommand, RawRobotState, RobotCommand,
};

/// Default maximum path deviation for the Move command.
pub const DEFAULT_DEVIATION_TRANSLATION: f64 = 10.0;
pub const DEFAULT_DEVIATION_ROTATION: f64 = 3.12;
pub const DEFAULT_DEVIATION_ELBOW: f64 = 2.0 * std::f64::consts::PI;

/// Configuration for the control loop.
#[derive(Debug, Clone)]
pub struct ControlLoopConfig {
    pub limit_rate: bool,
    pub cutoff_frequency: f64,
}

impl Default for ControlLoopConfig {
    fn default() -> Self {
        Self {
            limit_rate: true,
            cutoff_frequency: lowpass_filter::DEFAULT_CUTOFF_FREQUENCY,
        }
    }
}

/// Runs the 1kHz control loop for motion generation with an internal controller.
///
/// The `motion_callback` receives the current robot state and elapsed time since the last call,
/// and returns a `MotionResult<M>` — either `Continue(command)` or `Break(command)` (finished).
///
/// The motion command is filtered and rate-limited before being sent to the robot.
pub fn run_motion_loop<M, F>(
    network: &mut Network,
    controller_mode: ControllerMode,
    config: &ControlLoopConfig,
    mut motion_callback: F,
) -> FrankaResult<Vec<crate::logging::LogEntry>>
where
    M: MotionType,
    F: FnMut(&RobotState, Duration) -> MotionResult<M>,
{
    let motion_id = start_motion(network, controller_mode, M::motion_generator_mode())?;

    let mut logger = Logger::new(Logger::DEFAULT_CAPACITY);
    let mut filter_state = FilterState::new();

    let result = motion_loop_inner(
        network,
        motion_id,
        config,
        &mut motion_callback,
        &mut logger,
        &mut filter_state,
    );

    match &result {
        Ok(()) => {
            finish_motion(network, motion_id)?;
            Ok(logger.flush())
        }
        Err(_) => {
            let _ = cancel_motion(network, motion_id);
            result.map(|()| logger.flush())
        }
    }
}

/// Runs the 1kHz control loop for combined motion + torque control.
///
/// Both callbacks receive the current state and time step. The motion callback produces
/// motion commands, while the control callback produces torque commands.
pub fn run_motion_with_control_loop<M, MF, CF>(
    network: &mut Network,
    config: &ControlLoopConfig,
    mut motion_callback: MF,
    mut control_callback: CF,
) -> FrankaResult<Vec<crate::logging::LogEntry>>
where
    M: MotionType,
    MF: FnMut(&RobotState, Duration) -> MotionResult<M>,
    CF: FnMut(&RobotState, Duration) -> MotionResult<Torques>,
{
    let motion_id = start_motion(
        network,
        ControllerMode::ExternalController,
        M::motion_generator_mode(),
    )?;

    let mut logger = Logger::new(Logger::DEFAULT_CAPACITY);
    let mut filter_state = FilterState::new();

    let result = combined_loop_inner(
        network,
        motion_id,
        config,
        &mut motion_callback,
        &mut control_callback,
        &mut logger,
        &mut filter_state,
    );

    match &result {
        Ok(()) => {
            finish_motion(network, motion_id)?;
            Ok(logger.flush())
        }
        Err(_) => {
            let _ = cancel_motion(network, motion_id);
            result.map(|()| logger.flush())
        }
    }
}

/// Runs the 1kHz control loop for torque-only control (no motion generation).
pub fn run_torque_loop<F>(
    network: &mut Network,
    config: &ControlLoopConfig,
    mut control_callback: F,
) -> FrankaResult<Vec<crate::logging::LogEntry>>
where
    F: FnMut(&RobotState, Duration) -> MotionResult<Torques>,
{
    let motion_id = start_motion(
        network,
        ControllerMode::ExternalController,
        MotionGeneratorMode::None,
    )?;

    let mut logger = Logger::new(Logger::DEFAULT_CAPACITY);

    let result =
        torque_loop_inner(network, motion_id, config, &mut control_callback, &mut logger);

    match &result {
        Ok(()) => {
            finish_motion(network, motion_id)?;
            Ok(logger.flush())
        }
        Err(_) => {
            let _ = cancel_motion(network, motion_id);
            result.map(|()| logger.flush())
        }
    }
}

// --- Internal loop implementations ---

fn motion_loop_inner<M, F>(
    network: &mut Network,
    motion_id: u32,
    config: &ControlLoopConfig,
    motion_callback: &mut F,
    logger: &mut Logger,
    filter_state: &mut FilterState,
) -> FrankaResult<()>
where
    M: MotionType,
    F: FnMut(&RobotState, Duration) -> MotionResult<M>,
{
    let mut state = receive_robot_state(network)?;
    check_motion_error(&state, motion_id, network)?;

    let mut previous_time = state.time;

    loop {
        let time_step = state.time.saturating_sub(previous_time);
        let motion_result = motion_callback(&state, time_step);
        let motion_command =
            process_motion_command(&motion_result, &state, config, filter_state)?;

        let finished = is_finished(&motion_result);

        let robot_cmd = build_robot_command(
            0, // message_id set by caller in real implementation
            Some(&motion_command),
            None,
            finished,
            false,
        );

        logger.log(state.clone(), Some(robot_cmd));

        if finished {
            send_robot_command(network, &robot_cmd)?;
            return Ok(());
        }

        send_robot_command(network, &robot_cmd)?;
        previous_time = state.time;
        state = receive_robot_state(network)?;
        check_motion_error(&state, motion_id, network)?;
    }
}

fn combined_loop_inner<M, MF, CF>(
    network: &mut Network,
    motion_id: u32,
    config: &ControlLoopConfig,
    motion_callback: &mut MF,
    control_callback: &mut CF,
    logger: &mut Logger,
    filter_state: &mut FilterState,
) -> FrankaResult<()>
where
    M: MotionType,
    MF: FnMut(&RobotState, Duration) -> MotionResult<M>,
    CF: FnMut(&RobotState, Duration) -> MotionResult<Torques>,
{
    let mut state = receive_robot_state(network)?;
    check_motion_error(&state, motion_id, network)?;

    let mut previous_time = state.time;

    loop {
        let time_step = state.time.saturating_sub(previous_time);

        let control_result = control_callback(&state, time_step);
        let control_command = process_torque_command(&control_result, &state, config)?;

        let motion_result = motion_callback(&state, time_step);
        let motion_command =
            process_motion_command(&motion_result, &state, config, filter_state)?;

        let motion_finished = is_finished(&motion_result);
        let control_finished = is_finished(&control_result);
        let finished = motion_finished || control_finished;

        let robot_cmd = build_robot_command(
            0,
            Some(&motion_command),
            Some(&control_command),
            motion_finished,
            control_finished,
        );

        logger.log(state.clone(), Some(robot_cmd));

        if finished {
            send_robot_command(network, &robot_cmd)?;
            return Ok(());
        }

        send_robot_command(network, &robot_cmd)?;
        previous_time = state.time;
        state = receive_robot_state(network)?;
        check_motion_error(&state, motion_id, network)?;
    }
}

fn torque_loop_inner<F>(
    network: &mut Network,
    motion_id: u32,
    config: &ControlLoopConfig,
    control_callback: &mut F,
    logger: &mut Logger,
) -> FrankaResult<()>
where
    F: FnMut(&RobotState, Duration) -> MotionResult<Torques>,
{
    let mut state = receive_robot_state(network)?;
    check_motion_error(&state, motion_id, network)?;

    let mut previous_time = state.time;

    loop {
        let time_step = state.time.saturating_sub(previous_time);
        let control_result = control_callback(&state, time_step);
        let control_command = process_torque_command(&control_result, &state, config)?;

        let finished = is_finished(&control_result);

        let robot_cmd = build_robot_command(0, None, Some(&control_command), false, finished);

        logger.log(state.clone(), Some(robot_cmd));

        if finished {
            send_robot_command(network, &robot_cmd)?;
            return Ok(());
        }

        send_robot_command(network, &robot_cmd)?;
        previous_time = state.time;
        state = receive_robot_state(network)?;
        check_motion_error(&state, motion_id, network)?;
    }
}

// --- Motion command processing with filtering and rate limiting ---

struct FilterState {
    initialized: bool,
}

impl FilterState {
    fn new() -> Self {
        Self { initialized: false }
    }
}

fn process_motion_command<M: MotionType>(
    result: &MotionResult<M>,
    state: &RobotState,
    config: &ControlLoopConfig,
    filter_state: &mut FilterState,
) -> FrankaResult<MotionGeneratorCommand> {
    let motion = motion_value(result);
    let mut cmd = MotionGeneratorCommand {
        q_c: [0.0; 7],
        dq_c: [0.0; 7],
        o_t_ee_c: [0.0; 16],
        o_dp_ee_c: [0.0; 6],
        elbow_c: [0.0; 2],
        valid_elbow: 0,
        motion_generation_finished: 0,
    };

    convert_motion(&motion, state, config, filter_state, &mut cmd)?;
    Ok(cmd)
}

fn process_torque_command(
    result: &MotionResult<Torques>,
    state: &RobotState,
    config: &ControlLoopConfig,
) -> FrankaResult<ControllerCommand> {
    let torques = motion_value(result);
    let mut tau_j_d: [f64; 7] = *torques;

    if config.cutoff_frequency < MAX_CUTOFF_FREQUENCY {
        tau_j_d = lowpass_filter::lowpass_filter_joints(
            DELTA_T,
            &tau_j_d,
            &state.tau_j_d,
            config.cutoff_frequency,
        );
    }

    if config.limit_rate {
        tau_j_d =
            rate_limiting::limit_rate_torques(&rate_limiting::MAX_TORQUE_RATE, &tau_j_d, &state.tau_j_d);
    }

    check_finite_joints(&tau_j_d)?;

    Ok(ControllerCommand {
        tau_j_d,
        torque_command_finished: 0,
    })
}

/// Trait-driven motion conversion — specializes filtering and rate limiting per motion type.
fn convert_motion<M: MotionType>(
    motion: &M,
    state: &RobotState,
    config: &ControlLoopConfig,
    filter_state: &mut FilterState,
    cmd: &mut MotionGeneratorCommand,
) -> FrankaResult<()> {
    let mode = M::motion_generator_mode();
    match mode {
        MotionGeneratorMode::JointPosition => {
            convert_joint_positions(motion, state, config, filter_state, cmd)
        }
        MotionGeneratorMode::JointVelocity => {
            convert_joint_velocities(motion, state, config, cmd)
        }
        MotionGeneratorMode::CartesianPosition => {
            convert_cartesian_pose(motion, state, config, filter_state, cmd)
        }
        MotionGeneratorMode::CartesianVelocity => {
            convert_cartesian_velocities(motion, state, config, cmd)
        }
        _ => Err(FrankaError::InvalidOperation {
            message: "invalid motion generator mode for motion command".into(),
        }),
    }
}

fn convert_joint_positions<M: MotionType>(
    motion: &M,
    state: &RobotState,
    config: &ControlLoopConfig,
    filter_state: &mut FilterState,
    cmd: &mut MotionGeneratorCommand,
) -> FrankaResult<()> {
    let positions: &JointPositions =
        unsafe { &*(motion as *const M as *const JointPositions) };
    let mut q_c: [f64; 7] = **positions;

    let reference = if !filter_state.initialized {
        filter_state.initialized = true;
        q_c
    } else {
        state.q_d
    };

    if config.cutoff_frequency < MAX_CUTOFF_FREQUENCY {
        q_c = lowpass_filter::lowpass_filter_joints(
            DELTA_T,
            &q_c,
            &reference,
            config.cutoff_frequency,
        );
    }

    if config.limit_rate {
        q_c = rate_limiting::limit_rate_joint_positions(
            &rate_limiting::MAX_JOINT_ACCELERATION,
            &rate_limiting::MAX_JOINT_ACCELERATION,
            &rate_limiting::MAX_JOINT_ACCELERATION,
            &rate_limiting::MAX_JOINT_JERK,
            &q_c,
            &reference,
            &state.dq_d,
            &state.ddq_d,
        );
    }

    check_finite_joints(&q_c)?;
    cmd.q_c = q_c;
    Ok(())
}

fn convert_joint_velocities<M: MotionType>(
    motion: &M,
    state: &RobotState,
    config: &ControlLoopConfig,
    cmd: &mut MotionGeneratorCommand,
) -> FrankaResult<()> {
    let velocities: &JointVelocities =
        unsafe { &*(motion as *const M as *const JointVelocities) };
    let mut dq_c: [f64; 7] = **velocities;

    if config.cutoff_frequency < MAX_CUTOFF_FREQUENCY {
        dq_c = lowpass_filter::lowpass_filter_joints(
            DELTA_T,
            &dq_c,
            &state.dq_d,
            config.cutoff_frequency,
        );
    }

    if config.limit_rate {
        dq_c = rate_limiting::limit_rate_joint_velocities(
            &rate_limiting::MAX_JOINT_ACCELERATION,
            &rate_limiting::MAX_JOINT_ACCELERATION,
            &rate_limiting::MAX_JOINT_ACCELERATION,
            &rate_limiting::MAX_JOINT_JERK,
            &dq_c,
            &state.dq_d,
            &state.ddq_d,
        );
    }

    check_finite_joints(&dq_c)?;
    cmd.dq_c = dq_c;
    Ok(())
}

fn convert_cartesian_pose<M: MotionType>(
    motion: &M,
    state: &RobotState,
    config: &ControlLoopConfig,
    filter_state: &mut FilterState,
    cmd: &mut MotionGeneratorCommand,
) -> FrankaResult<()> {
    let pose: &CartesianPose =
        unsafe { &*(motion as *const M as *const CartesianPose) };

    let mut o_t_ee_c = pose.to_column_major();

    let reference_pose = if !filter_state.initialized {
        filter_state.initialized = true;
        o_t_ee_c
    } else {
        state.o_t_ee_c
    };

    if config.cutoff_frequency < MAX_CUTOFF_FREQUENCY {
        o_t_ee_c = lowpass_filter::cartesian_lowpass_filter(
            DELTA_T,
            &o_t_ee_c,
            &reference_pose,
            config.cutoff_frequency,
        );
    }

    if config.limit_rate {
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

    check_finite_array(&o_t_ee_c)?;
    cmd.o_t_ee_c = o_t_ee_c;

    if let Some(elbow) = pose.elbow {
        cmd.valid_elbow = 1;
        let mut elbow_c = elbow;
        let reference_elbow = if filter_state.initialized {
            state.elbow_c
        } else {
            elbow_c
        };

        if config.cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            elbow_c[0] = lowpass_filter::lowpass_filter(
                DELTA_T,
                elbow_c[0],
                reference_elbow[0],
                config.cutoff_frequency,
            );
        }

        if config.limit_rate {
            elbow_c[0] = rate_limiting::limit_rate_velocity(
                rate_limiting::MAX_ELBOW_VELOCITY,
                -rate_limiting::MAX_ELBOW_VELOCITY,
                rate_limiting::MAX_ELBOW_ACCELERATION,
                rate_limiting::MAX_ELBOW_JERK,
                elbow_c[0],
                reference_elbow[0],
                state.delbow_c[0],
            );
        }
        cmd.elbow_c = elbow_c;
    }

    Ok(())
}

fn convert_cartesian_velocities<M: MotionType>(
    motion: &M,
    state: &RobotState,
    config: &ControlLoopConfig,
    cmd: &mut MotionGeneratorCommand,
) -> FrankaResult<()> {
    let velocities: &CartesianVelocities =
        unsafe { &*(motion as *const M as *const CartesianVelocities) };

    let mut o_dp_ee_c = [
        velocities.linear.x, velocities.linear.y, velocities.linear.z,
        velocities.angular.x, velocities.angular.y, velocities.angular.z,
    ];

    if config.cutoff_frequency < MAX_CUTOFF_FREQUENCY {
        for (i, val) in o_dp_ee_c.iter_mut().enumerate() {
            *val = lowpass_filter::lowpass_filter(
                DELTA_T,
                *val,
                state.o_dp_ee_c[i],
                config.cutoff_frequency,
            );
        }
    }

    if config.limit_rate {
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
    cmd.o_dp_ee_c = o_dp_ee_c;

    if let Some(elbow) = velocities.elbow {
        cmd.valid_elbow = 1;
        let mut elbow_c = elbow;

        if config.cutoff_frequency < MAX_CUTOFF_FREQUENCY {
            elbow_c[0] = lowpass_filter::lowpass_filter(
                DELTA_T,
                elbow_c[0],
                state.elbow_c[0],
                config.cutoff_frequency,
            );
        }

        if config.limit_rate {
            elbow_c[0] = rate_limiting::limit_rate_velocity(
                rate_limiting::MAX_ELBOW_VELOCITY,
                -rate_limiting::MAX_ELBOW_VELOCITY,
                rate_limiting::MAX_ELBOW_ACCELERATION,
                rate_limiting::MAX_ELBOW_JERK,
                elbow_c[0],
                state.elbow_c[0],
                state.delbow_c[0],
            );
        }
        cmd.elbow_c = elbow_c;
    }

    Ok(())
}

// --- Network helpers ---

pub(crate) fn start_motion(
    network: &mut Network,
    controller_mode: ControllerMode,
    motion_generator_mode: MotionGeneratorMode,
) -> FrankaResult<u32> {
    let request = robot::MoveRequest {
        controller_mode: controller_mode as u32,
        motion_generator_mode: motion_generator_mode as u32,
        maximum_path_deviation_translation: DEFAULT_DEVIATION_TRANSLATION,
        maximum_path_deviation_rotation: DEFAULT_DEVIATION_ROTATION,
        maximum_path_deviation_elbow: DEFAULT_DEVIATION_ELBOW,
        maximum_goal_pose_deviation_translation: DEFAULT_DEVIATION_TRANSLATION,
        maximum_goal_pose_deviation_rotation: DEFAULT_DEVIATION_ROTATION,
        maximum_goal_pose_deviation_elbow: DEFAULT_DEVIATION_ELBOW,
        use_async_motion_generator: 0,
        maximum_velocity: [0.0; 7],
    };

    let payload = struct_to_bytes(&request);
    let command_id = network.tcp_send_request(robot::Command::Move as u32, &payload)?;
    let response = network.tcp_blocking_receive_response(command_id)?;

    if response.len() <= robot::CommandHeader::SIZE {
        return Err(FrankaError::Protocol {
            message: "Move response too short".into(),
        });
    }

    let status = response[robot::CommandHeader::SIZE];
    match robot::MoveStatus::from_u8(status) {
        Some(robot::MoveStatus::Success) | Some(robot::MoveStatus::MotionStarted) => {
            Ok(command_id)
        }
        Some(s) => Err(FrankaError::Command {
            message: format!("Move command rejected: {s:?}"),
        }),
        None => Err(FrankaError::Protocol {
            message: format!("unknown Move status: {status}"),
        }),
    }
}

pub(crate) fn finish_motion(network: &mut Network, _motion_id: u32) -> FrankaResult<()> {
    let command_id = network.tcp_send_request(robot::Command::StopMove as u32, &[])?;
    let response = network.tcp_blocking_receive_response(command_id)?;

    if response.len() <= robot::CommandHeader::SIZE {
        return Err(FrankaError::Protocol {
            message: "StopMove response too short".into(),
        });
    }

    Ok(())
}

fn cancel_motion(network: &mut Network, _motion_id: u32) -> FrankaResult<()> {
    let command_id = network.tcp_send_request(robot::Command::StopMove as u32, &[])?;
    let _ = network.tcp_blocking_receive_response(command_id);
    Ok(())
}

fn receive_robot_state(network: &Network) -> FrankaResult<RobotState> {
    let mut buf = [0u8; RawRobotState::SIZE + 128];
    let n = network.udp_blocking_receive(&mut buf)?;

    if n < RawRobotState::SIZE {
        return Err(FrankaError::Protocol {
            message: format!(
                "UDP state packet too small: got {n} bytes, expected at least {}",
                RawRobotState::SIZE
            ),
        });
    }

    let raw = unsafe { RawRobotState::from_bytes(&buf[..n]) };
    Ok(raw.to_robot_state())
}

fn send_robot_command(network: &Network, cmd: &RobotCommand) -> FrankaResult<()> {
    let bytes = struct_to_bytes(cmd);
    network.udp_send(&bytes)
}

fn check_motion_error(
    state: &RobotState,
    _motion_id: u32,
    _network: &mut Network,
) -> FrankaResult<()> {
    if !state.current_errors.is_empty() {
        return Err(FrankaError::Control {
            message: format!("robot reported errors: {:?}", state.current_errors),
            log: Vec::new(),
        });
    }
    Ok(())
}

fn build_robot_command(
    message_id: u64,
    motion: Option<&MotionGeneratorCommand>,
    control: Option<&ControllerCommand>,
    motion_finished: bool,
    control_finished: bool,
) -> RobotCommand {
    let mut motion_cmd = motion.copied().unwrap_or(MotionGeneratorCommand {
        q_c: [0.0; 7],
        dq_c: [0.0; 7],
        o_t_ee_c: [0.0; 16],
        o_dp_ee_c: [0.0; 6],
        elbow_c: [0.0; 2],
        valid_elbow: 0,
        motion_generation_finished: 0,
    });

    let mut control_cmd = control.copied().unwrap_or(ControllerCommand {
        tau_j_d: [0.0; 7],
        torque_command_finished: 0,
    });

    if motion_finished {
        motion_cmd.motion_generation_finished = 1;
    }
    if control_finished {
        control_cmd.torque_command_finished = 1;
    }

    RobotCommand {
        message_id,
        motion: motion_cmd,
        control: control_cmd,
    }
}

// --- Validation helpers ---

fn check_finite_joints(values: &[f64; 7]) -> FrankaResult<()> {
    for (i, &v) in values.iter().enumerate() {
        if !v.is_finite() {
            return Err(FrankaError::Realtime {
                message: format!("joint {i} command is not finite: {v}"),
            });
        }
    }
    Ok(())
}

fn check_finite_array<const N: usize>(values: &[f64; N]) -> FrankaResult<()> {
    for (i, &v) in values.iter().enumerate() {
        if !v.is_finite() {
            return Err(FrankaError::Realtime {
                message: format!("command element {i} is not finite: {v}"),
            });
        }
    }
    Ok(())
}

fn struct_to_bytes<T: Copy>(value: &T) -> Vec<u8> {
    let size = std::mem::size_of::<T>();
    let mut bytes = vec![0u8; size];
    unsafe {
        std::ptr::copy_nonoverlapping(value as *const T as *const u8, bytes.as_mut_ptr(), size);
    }
    bytes
}
