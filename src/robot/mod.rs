pub mod config;

use std::time::Duration;

use crate::constants::ROBOT_COMMAND_PORT;
use crate::control_loop;
use crate::control_types::{MotionResult, MotionType};
use crate::errors::{FrankaError, FrankaResult};
use crate::logging::LogEntry;
use crate::network::{self, Network, NetworkConfig};
use crate::robot_state::RobotState;
use crate::types::{
    CartesianPose, CartesianVelocities, ControllerMode, JointPositions, JointVelocities,
    RealtimeConfig, Torques,
};
use crate::wire::robot::{
    self, RawRobotState, SetCartesianImpedanceRequest, SetEeToKRequest, SetGuidingModeRequest,
    SetJointImpedanceRequest, SetNeToEeRequest,
};

use self::config::{CollisionConfig, LoadConfig, MotionConfig};

/// Main interface for connecting to and controlling a Franka robot.
///
/// The `Robot` struct owns the network connection and provides:
/// - State reading (`read_once`, `read`)
/// - Configuration commands (`set_collision_behavior`, `set_joint_impedance`, etc.)
/// - Motion control via callbacks (`control_joint_positions`, `control_torques`, etc.)
///
/// Only one control/motion loop can be active at a time — this is enforced at the
/// type level via `&mut self`.
pub struct Robot {
    network: Network,
    server_version: u16,
    realtime_config: RealtimeConfig,
}

impl Robot {
    /// Connect to a Franka robot at the given IP address or hostname.
    ///
    /// Performs the TCP connection and protocol version handshake.
    pub fn connect(address: &str) -> FrankaResult<Self> {
        Self::connect_with_config(address, RealtimeConfig::Enforce)
    }

    /// Connect with explicit realtime configuration.
    pub fn connect_with_config(
        address: &str,
        realtime_config: RealtimeConfig,
    ) -> FrankaResult<Self> {
        let net_config = NetworkConfig::default();
        let mut network = Network::connect(address, ROBOT_COMMAND_PORT, &net_config)?;
        let server_version = network::connect_robot(&mut network)?;

        Ok(Self {
            network,
            server_version,
            realtime_config,
        })
    }

    /// Returns the protocol version reported by the robot server.
    pub fn server_version(&self) -> u16 {
        self.server_version
    }

    /// Returns the realtime configuration.
    pub fn realtime_config(&self) -> RealtimeConfig {
        self.realtime_config
    }

    // === State Reading ===

    /// Read a single robot state from the UDP stream.
    ///
    /// Blocks until a state packet is received.
    pub fn read_once(&self) -> FrankaResult<RobotState> {
        let mut buf = [0u8; RawRobotState::SIZE + 128];
        let n = self.network.udp_blocking_receive(&mut buf)?;

        if n < RawRobotState::SIZE {
            return Err(FrankaError::Protocol {
                message: format!(
                    "UDP state packet too small: got {n}, expected {}",
                    RawRobotState::SIZE
                ),
            });
        }

        let raw = unsafe { RawRobotState::from_bytes(&buf[..n]) };
        Ok(raw.to_robot_state())
    }

    /// Continuously read robot state, calling `callback` for each update.
    ///
    /// The loop runs until the callback returns `false`.
    pub fn read<F>(&self, mut callback: F) -> FrankaResult<()>
    where
        F: FnMut(&RobotState) -> bool,
    {
        loop {
            let state = self.read_once()?;
            if !callback(&state) {
                return Ok(());
            }
        }
    }

    // === Motion Control (Callback-based) ===

    /// Run a joint position motion generator with the robot's internal controller.
    ///
    /// The callback receives the current robot state and time since last call,
    /// and returns `ControlFlow::Continue(positions)` or `ControlFlow::Break(positions)`.
    pub fn control_joint_positions<F>(
        &mut self,
        motion_config: &MotionConfig,
        callback: F,
    ) -> FrankaResult<Vec<LogEntry>>
    where
        F: FnMut(&RobotState, Duration) -> MotionResult<JointPositions>,
    {
        control_loop::run_motion_loop(
            &mut self.network,
            motion_config.controller_mode,
            &motion_config.to_control_loop_config(),
            callback,
        )
    }

    /// Run a joint velocity motion generator with the robot's internal controller.
    pub fn control_joint_velocities<F>(
        &mut self,
        motion_config: &MotionConfig,
        callback: F,
    ) -> FrankaResult<Vec<LogEntry>>
    where
        F: FnMut(&RobotState, Duration) -> MotionResult<JointVelocities>,
    {
        control_loop::run_motion_loop(
            &mut self.network,
            motion_config.controller_mode,
            &motion_config.to_control_loop_config(),
            callback,
        )
    }

    /// Run a Cartesian pose motion generator with the robot's internal controller.
    pub fn control_cartesian_pose<F>(
        &mut self,
        motion_config: &MotionConfig,
        callback: F,
    ) -> FrankaResult<Vec<LogEntry>>
    where
        F: FnMut(&RobotState, Duration) -> MotionResult<CartesianPose>,
    {
        control_loop::run_motion_loop(
            &mut self.network,
            motion_config.controller_mode,
            &motion_config.to_control_loop_config(),
            callback,
        )
    }

    /// Run a Cartesian velocity motion generator with the robot's internal controller.
    pub fn control_cartesian_velocities<F>(
        &mut self,
        motion_config: &MotionConfig,
        callback: F,
    ) -> FrankaResult<Vec<LogEntry>>
    where
        F: FnMut(&RobotState, Duration) -> MotionResult<CartesianVelocities>,
    {
        control_loop::run_motion_loop(
            &mut self.network,
            motion_config.controller_mode,
            &motion_config.to_control_loop_config(),
            callback,
        )
    }

    /// Run a torque-only control loop (no motion generation).
    pub fn control_torques<F>(
        &mut self,
        motion_config: &MotionConfig,
        callback: F,
    ) -> FrankaResult<Vec<LogEntry>>
    where
        F: FnMut(&RobotState, Duration) -> MotionResult<Torques>,
    {
        control_loop::run_torque_loop(
            &mut self.network,
            &motion_config.to_control_loop_config(),
            callback,
        )
    }

    /// Run combined motion + torque control.
    ///
    /// The motion callback generates the desired trajectory, while the torque callback
    /// provides additional joint-level torque commands.
    pub fn control_motion_with_torques<M, MF, CF>(
        &mut self,
        motion_config: &MotionConfig,
        motion_callback: MF,
        control_callback: CF,
    ) -> FrankaResult<Vec<LogEntry>>
    where
        M: MotionType,
        MF: FnMut(&RobotState, Duration) -> MotionResult<M>,
        CF: FnMut(&RobotState, Duration) -> MotionResult<Torques>,
    {
        control_loop::run_motion_with_control_loop(
            &mut self.network,
            &motion_config.to_control_loop_config(),
            motion_callback,
            control_callback,
        )
    }

    // === Configuration Commands ===

    /// Set collision behavior thresholds.
    pub fn set_collision_behavior(&mut self, config: &CollisionConfig) -> FrankaResult<()> {
        let request = config.to_request();
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.set_collision_behavior(&request)
    }

    /// Set joint impedance values for the internal controller.
    ///
    /// Values in Nm/rad, range [0, 14250] per joint.
    pub fn set_joint_impedance(&mut self, k_theta: [f64; 7]) -> FrankaResult<()> {
        let request = SetJointImpedanceRequest { k_theta };
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.set_joint_impedance(&request)
    }

    /// Set Cartesian impedance values for the internal controller.
    ///
    /// Values for (x, y, z, roll, pitch, yaw).
    /// Linear: [10, 3000] N/m. Rotational: [1, 300] Nm/rad.
    pub fn set_cartesian_impedance(&mut self, k_x: [f64; 6]) -> FrankaResult<()> {
        let request = SetCartesianImpedanceRequest { k_x };
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.set_cartesian_impedance(&request)
    }

    /// Set guiding mode (which axes are free for hand-guiding).
    ///
    /// `guiding_mode[i] = true` means axis i is unlocked for guiding.
    /// `elbow_free` controls whether the elbow is free in guiding mode.
    pub fn set_guiding_mode(
        &mut self,
        guiding_mode: [bool; 6],
        elbow_free: bool,
    ) -> FrankaResult<()> {
        let mut mode_bytes = [0u8; 6];
        for (i, &v) in guiding_mode.iter().enumerate() {
            mode_bytes[i] = v as u8;
        }
        let request = SetGuidingModeRequest {
            guiding_mode: mode_bytes,
            nullspace: elbow_free as u8,
        };
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.set_guiding_mode(&request)
    }

    /// Set the end-effector to stiffness frame transformation (EE_T_K).
    ///
    /// `ee_t_k` is a column-major 4x4 homogeneous transformation matrix.
    pub fn set_k_frame(&mut self, ee_t_k: [f64; 16]) -> FrankaResult<()> {
        let request = SetEeToKRequest { ee_t_k };
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.set_ee_to_k(&request)
    }

    /// Set the nominal end-effector to end-effector frame transformation (NE_T_EE).
    ///
    /// `ne_t_ee` is a column-major 4x4 homogeneous transformation matrix.
    pub fn set_ee_frame(&mut self, ne_t_ee: [f64; 16]) -> FrankaResult<()> {
        let request = SetNeToEeRequest { ne_t_ee };
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.set_ne_to_ee(&request)
    }

    /// Set external load (payload) parameters.
    pub fn set_load(&mut self, config: &LoadConfig) -> FrankaResult<()> {
        let request = config.to_request();
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.set_load(&request)
    }

    /// Trigger automatic error recovery (e.g., after a collision).
    pub fn automatic_error_recovery(&mut self) -> FrankaResult<()> {
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.automatic_error_recovery()
    }

    /// Stop all currently running motions.
    pub fn stop(&mut self) -> FrankaResult<()> {
        let command_id = self
            .network
            .tcp_send_request(robot::Command::StopMove as u32, &[])?;
        let response = self.network.tcp_blocking_receive_response(command_id)?;

        if response.len() <= robot::CommandHeader::SIZE {
            return Err(FrankaError::Protocol {
                message: "StopMove response too short".into(),
            });
        }

        let status = response[robot::CommandHeader::SIZE];
        match status {
            0 => Ok(()),
            _ => Err(FrankaError::Command {
                message: format!("StopMove failed with status {status}"),
            }),
        }
    }

    /// Request the robot model URDF as a string.
    pub fn get_robot_model(&mut self) -> FrankaResult<String> {
        let mut cmd = network::RobotCommand::new(&mut self.network);
        cmd.get_robot_model()
    }

    // === Active Control (non-callback interface) ===

    /// Start a torque control session.
    ///
    /// Returns an `ActiveTorqueControl` handle that allows reading state
    /// and writing torque commands without a callback.
    pub fn start_torque_control(
        &mut self,
    ) -> FrankaResult<crate::active_control::ActiveTorqueControl<'_>> {
        crate::active_control::ActiveTorqueControl::start(&mut self.network)
    }

    /// Start a motion control session with the given motion type.
    ///
    /// Returns an `ActiveMotionControl` handle for read/write control.
    pub fn start_motion_control<M: MotionType>(
        &mut self,
        controller_mode: ControllerMode,
    ) -> FrankaResult<crate::active_control::ActiveMotionControl<'_, M>> {
        crate::active_control::ActiveMotionControl::start(&mut self.network, controller_mode)
    }
}
