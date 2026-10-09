use std::panic::{self, AssertUnwindSafe};
use std::time::Duration;

use crate::constants::DELTA_T;
use crate::control_types::{is_finished, motion_value, MotionResult, MotionType};
use crate::errors::{FrankaError, FrankaResult};
use crate::joint_velocity_limits::JointVelocityLimits;
use crate::logging::Logger;
use crate::lowpass_filter;
use crate::motion_conversion::{self, FilterState};
use crate::network::Network;
use crate::rate_limiting;
use crate::robot_state::RobotState;
use crate::types::{ControllerMode, MotionGeneratorMode, Torques};
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
    joint_velocity_limits: &JointVelocityLimits,
    mut motion_callback: F,
) -> FrankaResult<Vec<crate::logging::LogEntry>>
where
    M: MotionType,
    F: FnMut(&RobotState, Duration) -> MotionResult<M>,
{
    // As libfranka's ControlLoop for motion-only callbacks: an external controller needs torques.
    if controller_mode == ControllerMode::ExternalController {
        return Err(FrankaError::InvalidOperation {
            message: "invalid controller mode for a motion-only control loop: ExternalController needs torques".into(),
        });
    }
    let motion_id = start_motion(network, controller_mode, M::motion_generator_mode())?;

    let mut logger = Logger::new(Logger::DEFAULT_CAPACITY);
    let mut filter_state = FilterState::new();

    run_and_finish(network, motion_id, |network| {
        motion_loop_inner(
            network,
            motion_id,
            config,
            joint_velocity_limits,
            &mut motion_callback,
            &mut logger,
            &mut filter_state,
        )
    })?;
    Ok(logger.flush())
}

/// Runs the 1kHz control loop for combined motion + torque control.
///
/// Both callbacks receive the current state and time step. The motion callback produces
/// motion commands, while the control callback produces torque commands.
pub fn run_motion_with_control_loop<M, MF, CF>(
    network: &mut Network,
    config: &ControlLoopConfig,
    joint_velocity_limits: &JointVelocityLimits,
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

    run_and_finish(network, motion_id, |network| {
        combined_loop_inner(
            network,
            motion_id,
            config,
            joint_velocity_limits,
            &mut motion_callback,
            &mut control_callback,
            &mut logger,
            &mut filter_state,
        )
    })?;
    Ok(logger.flush())
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

    run_and_finish(network, motion_id, |network| {
        torque_loop_inner(network, motion_id, config, &mut control_callback, &mut logger)
    })?;
    Ok(logger.flush())
}

/// Runs a started motion's control loop, then finishes the motion with the loop's final command,
/// as libfranka's `ControlLoop::loop` does. If the loop or the finish returns an error, or a
/// callback panics, the motion is cancelled (StopMove) before the error is returned or the panic
/// continues.
fn run_and_finish<L>(network: &mut Network, motion_id: u32, control_loop: L) -> FrankaResult<()>
where
    L: FnOnce(&mut Network) -> FrankaResult<RobotCommand>,
{
    // The network is only read again by cancel_motion, which needs no invariant that a panic
    // part-way through the loop could break.
    let outcome = panic::catch_unwind(AssertUnwindSafe(|| {
        control_loop(network).and_then(|finished_command| finish_motion(network, motion_id, finished_command))
    }));
    match outcome {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => {
            let _ = cancel_motion(network, motion_id);
            Err(error)
        }
        Err(panic_payload) => {
            let _ = cancel_motion(network, motion_id);
            panic::resume_unwind(panic_payload)
        }
    }
}

// --- Internal loop implementations ---

fn motion_loop_inner<M, F>(
    network: &mut Network,
    motion_id: u32,
    config: &ControlLoopConfig,
    joint_velocity_limits: &JointVelocityLimits,
    motion_callback: &mut F,
    logger: &mut Logger,
    filter_state: &mut FilterState,
) -> FrankaResult<RobotCommand>
where
    M: MotionType,
    F: FnMut(&RobotState, Duration) -> MotionResult<M>,
{
    // As libfranka's first updateMotion, which sends no command but checks the connection.
    network.tcp_throw_if_connection_closed()?;
    let mut state = receive_robot_state(network)?;
    check_motion_error(&state, motion_id, network)?;

    let mut previous_time = state.time;

    loop {
        let time_step = state.time.saturating_sub(previous_time);
        let motion_result = motion_callback(&state, time_step);
        let motion_command =
            process_motion_command(&motion_result, &state, config, joint_velocity_limits, filter_state)?;

        let finished = is_finished(&motion_result);

        let robot_cmd = build_robot_command(
            network.latest_state_message_id(),
            Some(&motion_command),
            None,
            finished,
            false,
        );

        logger.log(state.clone(), Some(robot_cmd));

        if finished {
            // Sent by finish_motion, as libfranka's ControlLoop leaves it to finishMotion.
            return Ok(robot_cmd);
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
    joint_velocity_limits: &JointVelocityLimits,
    motion_callback: &mut MF,
    control_callback: &mut CF,
    logger: &mut Logger,
    filter_state: &mut FilterState,
) -> FrankaResult<RobotCommand>
where
    M: MotionType,
    MF: FnMut(&RobotState, Duration) -> MotionResult<M>,
    CF: FnMut(&RobotState, Duration) -> MotionResult<Torques>,
{
    // As libfranka's first updateMotion, which sends no command but checks the connection.
    network.tcp_throw_if_connection_closed()?;
    let mut state = receive_robot_state(network)?;
    check_motion_error(&state, motion_id, network)?;

    let mut previous_time = state.time;
    let mut motion_command = motion_conversion::empty_command();

    loop {
        let time_step = state.time.saturating_sub(previous_time);

        let control_result = control_callback(&state, time_step);
        let control_command = process_torque_command(&control_result, &state, config)?;
        let mut finished = is_finished(&control_result);

        // As in libfranka's ControlLoop, the motion callback runs only while the control
        // callback has not finished; otherwise the previous motion command is kept.
        if !finished {
            let motion_result = motion_callback(&state, time_step);
            motion_command =
                process_motion_command(&motion_result, &state, config, joint_velocity_limits, filter_state)?;
            finished = is_finished(&motion_result);
        }

        // libfranka's finishMotion marks only the motion command as finished when one exists.
        let robot_cmd = build_robot_command(
            network.latest_state_message_id(),
            Some(&motion_command),
            Some(&control_command),
            finished,
            false,
        );

        logger.log(state.clone(), Some(robot_cmd));

        if finished {
            // Sent by finish_motion, as libfranka's ControlLoop leaves it to finishMotion.
            return Ok(robot_cmd);
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
) -> FrankaResult<RobotCommand>
where
    F: FnMut(&RobotState, Duration) -> MotionResult<Torques>,
{
    // As libfranka's first updateMotion, which sends no command but checks the connection.
    network.tcp_throw_if_connection_closed()?;
    let mut state = receive_robot_state(network)?;
    check_motion_error(&state, motion_id, network)?;

    let mut previous_time = state.time;

    loop {
        let time_step = state.time.saturating_sub(previous_time);
        let control_result = control_callback(&state, time_step);
        let control_command = process_torque_command(&control_result, &state, config)?;

        let finished = is_finished(&control_result);

        let robot_cmd = build_robot_command(
            network.latest_state_message_id(),
            None,
            Some(&control_command),
            false,
            finished,
        );

        logger.log(state.clone(), Some(robot_cmd));

        if finished {
            // Sent by finish_motion, as libfranka's ControlLoop leaves it to finishMotion.
            return Ok(robot_cmd);
        }

        send_robot_command(network, &robot_cmd)?;
        previous_time = state.time;
        state = receive_robot_state(network)?;
        check_motion_error(&state, motion_id, network)?;
    }
}

// --- Command processing with filtering and rate limiting ---

fn process_motion_command<M: MotionType>(
    result: &MotionResult<M>,
    state: &RobotState,
    config: &ControlLoopConfig,
    joint_velocity_limits: &JointVelocityLimits,
    filter_state: &mut FilterState,
) -> FrankaResult<MotionGeneratorCommand> {
    motion_value(result).control_loop_command(state, config, joint_velocity_limits, filter_state)
}

fn process_torque_command(
    result: &MotionResult<Torques>,
    state: &RobotState,
    config: &ControlLoopConfig,
) -> FrankaResult<ControllerCommand> {
    let torques = motion_value(result);
    let mut tau_j_d: [f64; 7] = *torques;

    // As libfranka's lowpassFilter and limitRate, non-finite inputs are rejected before filtering
    // and limiting: the limiter would otherwise turn an infinite torque into a finite maximum step.
    if motion_conversion::filters(config)? {
        check_finite_joints(&tau_j_d)?;
        if let Some(joint) = state.tau_j_d.iter().position(|value| !value.is_finite()) {
            return Err(FrankaError::Realtime {
                message: format!(
                    "joint {joint} of the robot's last desired torque is not finite: {}",
                    state.tau_j_d[joint]
                ),
            });
        }
        tau_j_d = lowpass_filter::lowpass_filter_joints(
            DELTA_T,
            &tau_j_d,
            &state.tau_j_d,
            config.cutoff_frequency,
        );
    }

    if config.limit_rate {
        check_finite_joints(&tau_j_d)?;
        tau_j_d =
            rate_limiting::limit_rate_torques(&rate_limiting::MAX_TORQUE_RATE, &tau_j_d, &state.tau_j_d);
    }

    check_finite_joints(&tau_j_d)?;

    Ok(ControllerCommand {
        tau_j_d,
        torque_command_finished: 0,
    })
}

// --- Network helpers ---

pub(crate) fn start_motion(
    network: &mut Network,
    controller_mode: ControllerMode,
    motion_generator_mode: MotionGeneratorMode,
) -> FrankaResult<u32> {
    // As libfranka's startMotion: refuse while a motion is still running.
    if motion_running(network) {
        return Err(FrankaError::Control {
            message: "attempted to start multiple motions".into(),
            log: Vec::new(),
        });
    }
    let requested = (motion_generator_mode as u8, controller_mode as u8);
    network.set_current_move_modes(Some(requested));

    let request = robot::MoveRequest {
        controller_mode: robot::MoveControllerMode::from(controller_mode) as u32,
        motion_generator_mode: robot::MoveMotionGeneratorMode::try_from(motion_generator_mode)? as u32,
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

    check_move_response(&response)?;

    // As libfranka's startMotion: read robot states until they report the requested modes,
    // stopping early if the robot already answers the Move again (an abort or immediate finish).
    // A rejection there is a control error, as in libfranka.
    while network.latest_state_modes() != requested {
        if let Some(response) = network.tcp_try_receive_response(command_id)? {
            check_move_response(&response).map_err(command_to_control)?;
            break;
        }
        network.tcp_throw_if_connection_closed()?;
        receive_robot_state(network)?;
    }
    Ok(command_id)
}

/// Converts a [`FrankaError::Command`] into a [`FrankaError::Control`] with the same message, as
/// libfranka rethrows a CommandException as a ControlException; other errors pass unchanged.
fn command_to_control(error: FrankaError) -> FrankaError {
    match error {
        FrankaError::Command { message } => FrankaError::Control { message, log: Vec::new() },
        other => other,
    }
}

/// Returns an error unless a Move response reports success or that the motion started.
fn check_move_response(response: &[u8]) -> FrankaResult<()> {
    if response.len() <= robot::CommandHeader::SIZE {
        return Err(FrankaError::Protocol {
            message: "Move response too short".into(),
        });
    }
    let status = response[robot::CommandHeader::SIZE];
    match robot::MoveStatus::from_u8(status) {
        Some(robot::MoveStatus::Success) | Some(robot::MoveStatus::MotionStarted) => Ok(()),
        Some(s) => Err(FrankaError::Command {
            message: format!("Move command rejected: {s:?}"),
        }),
        None => Err(FrankaError::Protocol {
            message: format!("unknown Move status: {status}"),
        }),
    }
}

/// Returns whether the latest robot state reports a running motion generator or external
/// controller (robot-state numbering: motion generator Idle 0 and None 5, controller
/// ExternalController 2).
fn motion_running(network: &Network) -> bool {
    let (motion_generator_mode, controller_mode) = network.latest_state_modes();
    (motion_generator_mode != MotionGeneratorMode::Idle as u8 && motion_generator_mode != MotionGeneratorMode::None as u8)
        || controller_mode == ControllerMode::ExternalController as u8
}

/// Finishes a motion as libfranka's finishMotion does: sends `finished_command` (whose finished
/// flag is set) once per received robot state while the robot still reports the motion running,
/// then waits for the Move response.
pub(crate) fn finish_motion(network: &mut Network, motion_id: u32, mut finished_command: RobotCommand) -> FrankaResult<()> {
    // As libfranka: nothing to finish (and no Move response to wait for) if nothing runs.
    if !motion_running(network) {
        network.set_current_move_modes(None);
        return Ok(());
    }
    while motion_running(network) {
        finished_command.message_id = network.latest_state_message_id();
        send_robot_command(network, &finished_command)?;
        receive_robot_state(network)?;
    }
    let response = network.tcp_blocking_receive_response(motion_id)?;
    if response.len() > robot::CommandHeader::SIZE
        && robot::MoveStatus::from_u8(response[robot::CommandHeader::SIZE]) == Some(robot::MoveStatus::ReflexAborted)
    {
        return Err(FrankaError::Control {
            message: "motion finished commanded, but the robot is still moving".into(),
            log: Vec::new(),
        });
    }
    check_move_response(&response).map_err(command_to_control)?;
    network.set_current_move_modes(None);
    Ok(())
}

/// Cancels a motion as libfranka's cancelMotion does: sends StopMove, reads robot states until the
/// motion is no longer running, then takes the Move response if it has arrived.
pub(crate) fn cancel_motion(network: &mut Network, motion_id: u32) -> FrankaResult<()> {
    // As libfranka: a closed connection cannot carry the StopMove.
    if !network.is_tcp_alive() {
        return Err(FrankaError::network("TCP connection is closed; cannot cancel the motion"));
    }
    let command_id = network.tcp_send_request(robot::Command::StopMove as u32, &[])?;
    let response = network.tcp_blocking_receive_response(command_id)?;
    if response.len() <= robot::CommandHeader::SIZE || response[robot::CommandHeader::SIZE] != 0 {
        return Err(FrankaError::Control {
            message: "StopMove command rejected".into(),
            log: Vec::new(),
        });
    }
    loop {
        receive_robot_state(network)?;
        if !motion_running(network) {
            break;
        }
    }
    let _ = network.tcp_try_receive_response(motion_id)?;
    network.set_current_move_modes(None);
    Ok(())
}

/// Returns the newest robot state whose message ID is greater than the last one recorded on
/// `network`, and records its ID.
///
/// States already queued on the socket are drained and the newest is kept; if none is newer than
/// the last recorded state, blocks until one arrives.
///
/// # Errors
/// Returns [`FrankaError::Protocol`] if a received datagram is not exactly the size of a robot
/// state, and [`FrankaError::Network`] if receiving fails or no datagram arrives within the
/// network's UDP timeout.
pub(crate) fn receive_robot_state(network: &Network) -> FrankaResult<RobotState> {
    let last_message_id = network.latest_state_message_id();
    let mut newest: Option<RawRobotState> = None;
    // One byte larger than a state, so an oversized datagram shows up as a wrong length instead
    // of being truncated to a valid-looking state.
    let mut buf = [0u8; RawRobotState::SIZE + 1];

    while let Some(n) = network.udp_try_receive(&mut buf)? {
        newest = newer_state(newest, last_message_id, decode_robot_state(&buf[..n])?);
    }
    while newest.is_none() {
        let n = network.udp_blocking_receive(&mut buf)?;
        newest = newer_state(newest, last_message_id, decode_robot_state(&buf[..n])?);
    }

    let raw = newest.expect("loop exits only once a newer state is kept");
    network.record_state(raw.message_id, raw.robot_mode, raw.motion_generator_mode, raw.controller_mode);
    Ok(raw.to_robot_state())
}

/// Returns `candidate` if its message ID exceeds that of `newest` (or `last_message_id` when there
/// is no `newest` yet), and `newest` otherwise.
fn newer_state(
    newest: Option<RawRobotState>,
    last_message_id: u64,
    candidate: RawRobotState,
) -> Option<RawRobotState> {
    let newest_message_id = newest.map_or(last_message_id, |state| state.message_id);
    if candidate.message_id > newest_message_id {
        Some(candidate)
    } else {
        newest
    }
}

/// Decodes one UDP datagram as a robot state.
///
/// # Errors
/// Returns [`FrankaError::Protocol`] if `datagram` is not exactly the size of a robot state.
fn decode_robot_state(datagram: &[u8]) -> FrankaResult<RawRobotState> {
    if datagram.len() != RawRobotState::SIZE {
        return Err(FrankaError::Protocol {
            message: format!(
                "UDP state packet has wrong size: got {} bytes, expected {}",
                datagram.len(),
                RawRobotState::SIZE
            ),
        });
    }
    Ok(unsafe { RawRobotState::from_bytes(datagram) })
}

/// Sends a robot command after checking that the robot has not closed the TCP connection, as
/// libfranka's updateMotion does.
pub(crate) fn send_robot_command(network: &Network, cmd: &RobotCommand) -> FrankaResult<()> {
    network.tcp_throw_if_connection_closed()?;
    let bytes = struct_to_bytes(cmd);
    network.udp_send(&bytes)
}

/// Returns an error once the robot has left the running motion, as libfranka's throwOnMotionError:
/// when the latest state's robot mode is not Move, or its modes differ from the running Move's,
/// the Move response is read to report why (an abort status), or a protocol error if it reports
/// none.
pub(crate) fn check_motion_error(state: &RobotState, motion_id: u32, network: &mut Network) -> FrankaResult<()> {
    let still_running = network.latest_robot_mode() == crate::types::RobotMode::Move as u8
        && Some(network.latest_state_modes()) == network.current_move_modes();
    if still_running {
        return Ok(());
    }
    let response = network.tcp_blocking_receive_response(motion_id)?;
    if let Err(error) = check_move_response(&response) {
        return Err(FrankaError::Control {
            message: format!("{error} (last motion errors: {:?})", state.last_motion_errors),
            log: Vec::new(),
        });
    }
    Err(FrankaError::Protocol {
        message: "unexpected reply to a Move command".into(),
    })
}


fn build_robot_command(
    message_id: u64,
    motion: Option<&MotionGeneratorCommand>,
    control: Option<&ControllerCommand>,
    motion_finished: bool,
    control_finished: bool,
) -> RobotCommand {
    let mut motion_cmd = motion.copied().unwrap_or_else(motion_conversion::empty_command);

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

pub(crate) fn check_finite_joints(values: &[f64; 7]) -> FrankaResult<()> {
    for (i, &v) in values.iter().enumerate() {
        if !v.is_finite() {
            return Err(FrankaError::Realtime {
                message: format!("joint {i} command is not finite: {v}"),
            });
        }
    }
    Ok(())
}

pub(crate) fn check_finite_array<const N: usize>(values: &[f64; N]) -> FrankaResult<()> {
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

#[cfg(test)]
mod tests {
    use std::net::{TcpListener, UdpSocket};
    use std::thread;

    use super::*;
    use crate::network::NetworkConfig;

    /// Connects a `Network` to a local accept-only TCP listener, and returns it with a UDP socket
    /// that stands in for the robot's state stream.
    fn connect_local() -> (Network, UdpSocket, thread::JoinHandle<()>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let tcp_server = thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            thread::sleep(Duration::from_secs(1));
            drop(stream);
        });
        let config = NetworkConfig {
            udp_timeout: Duration::from_secs(1),
            keepalive_enabled: false,
            ..Default::default()
        };
        let network = Network::connect("127.0.0.1", port, &config).unwrap();
        (network, UdpSocket::bind("127.0.0.1:0").unwrap(), tcp_server)
    }

    /// Sends a robot state datagram carrying `message_id` to `network`.
    fn send_state(robot: &UdpSocket, network: &Network, message_id: u64) {
        let mut datagram = vec![0u8; RawRobotState::SIZE];
        datagram[0..8].copy_from_slice(&message_id.to_ne_bytes());
        robot.send_to(&datagram, ("127.0.0.1", network.udp_port())).unwrap();
    }

    #[test]
    fn receive_robot_state_returns_newest_queued_state() {
        let (network, robot, tcp_server) = connect_local();
        for message_id in [5, 7, 6] {
            send_state(&robot, &network, message_id);
        }
        thread::sleep(Duration::from_millis(50));

        let state = receive_robot_state(&network).unwrap();
        assert_eq!(state.time, Duration::from_millis(7));
        assert_eq!(network.latest_state_message_id(), 7);
        tcp_server.join().unwrap();
    }

    #[test]
    fn receive_robot_state_skips_states_not_newer_than_last() {
        let (network, robot, tcp_server) = connect_local();
        send_state(&robot, &network, 7);
        thread::sleep(Duration::from_millis(50));
        receive_robot_state(&network).unwrap();

        // Only stale states are queued, so the call must wait for the newer one sent later.
        send_state(&robot, &network, 7);
        send_state(&robot, &network, 4);
        let late_sender = thread::spawn({
            let robot = robot.try_clone().unwrap();
            let udp_port = network.udp_port();
            move || {
                thread::sleep(Duration::from_millis(100));
                let mut datagram = vec![0u8; RawRobotState::SIZE];
                datagram[0..8].copy_from_slice(&10u64.to_ne_bytes());
                robot.send_to(&datagram, ("127.0.0.1", udp_port)).unwrap();
            }
        });

        let state = receive_robot_state(&network).unwrap();
        assert_eq!(state.time, Duration::from_millis(10));
        assert_eq!(network.latest_state_message_id(), 10);
        late_sender.join().unwrap();
        tcp_server.join().unwrap();
    }

    /// Returns a robot state decoded from an all-zero datagram (every desired torque zero).
    fn zero_state() -> RobotState {
        decode_robot_state(&[0u8; RawRobotState::SIZE]).unwrap().to_robot_state()
    }

    /// Returns a control-loop configuration with the given rate limiting and low-pass filter
    /// (100 Hz) settings.
    fn torque_config(limit_rate: bool, filter: bool) -> ControlLoopConfig {
        ControlLoopConfig {
            limit_rate,
            cutoff_frequency: if filter { lowpass_filter::DEFAULT_CUTOFF_FREQUENCY } else { lowpass_filter::MAX_CUTOFF_FREQUENCY },
        }
    }

    #[test]
    fn process_torque_command_rejects_non_finite_torque_in_every_configuration() {
        for bad_value in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            for (limit_rate, filter) in [(false, false), (true, false), (false, true), (true, true)] {
                let mut tau = [0.0; 7];
                tau[3] = bad_value;
                let result = process_torque_command(
                    &MotionResult::Continue(Torques::new(tau)),
                    &zero_state(),
                    &torque_config(limit_rate, filter),
                );
                assert!(
                    matches!(result, Err(FrankaError::Realtime { .. })),
                    "torque {bad_value}, limit_rate {limit_rate}, filter {filter}: {result:?}"
                );
            }
        }
    }

    #[test]
    fn process_torque_command_rejects_non_positive_cutoff_frequency() {
        for cutoff_frequency in [0.0, -10.0, f64::NEG_INFINITY] {
            let config = ControlLoopConfig { limit_rate: false, cutoff_frequency };
            let result = process_torque_command(
                &MotionResult::Continue(Torques::new([1.0; 7])),
                &zero_state(),
                &config,
            );
            assert!(
                matches!(result, Err(FrankaError::InvalidOperation { .. })),
                "cutoff {cutoff_frequency}: {result:?}"
            );
        }
    }

    #[test]
    fn process_torque_command_rejects_non_finite_last_desired_torque_when_filtering() {
        let mut state = zero_state();
        state.tau_j_d[0] = f64::NAN;
        let result = process_torque_command(
            &MotionResult::Continue(Torques::new([0.0; 7])),
            &state,
            &torque_config(false, true),
        );
        assert!(
            matches!(&result, Err(FrankaError::Realtime { message }) if message.contains("last desired torque")),
            "{result:?}"
        );
    }

    #[test]
    fn receive_robot_state_rejects_too_small_packet() {
        let (network, robot, tcp_server) = connect_local();
        let datagram = vec![0u8; RawRobotState::SIZE - 1];
        robot.send_to(&datagram, ("127.0.0.1", network.udp_port())).unwrap();

        let result = receive_robot_state(&network);
        assert!(
            matches!(&result, Err(FrankaError::Protocol { message }) if message.contains("wrong size")),
            "{result:?}"
        );
        assert_eq!(network.latest_state_message_id(), 0);
        tcp_server.join().unwrap();
    }
}
