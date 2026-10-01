/// Wire-format command enum matching research_interface::robot::Command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Command {
    Connect = 0,
    Move = 1,
    StopMove = 2,
    SetCollisionBehavior = 3,
    SetJointImpedance = 4,
    SetCartesianImpedance = 5,
    SetGuidingMode = 6,
    SetEeToK = 7,
    SetNeToEe = 8,
    SetLoad = 9,
    AutomaticErrorRecovery = 10,
    GetRobotModel = 11,
}

/// TCP message header for robot commands.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct CommandHeader {
    pub command: u32,
    pub command_id: u32,
    pub size: u32,
}

/// Connect request sent to the robot.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct ConnectRequest {
    pub version: u16,
    pub udp_port: u16,
}

/// Connect response from the robot.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct ConnectResponse {
    pub status: u8,
    pub version: u16,
}

/// Status codes for the Connect command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ConnectStatus {
    Success = 0,
    IncompatibleLibraryVersion = 1,
}

/// Move command request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct MoveRequest {
    pub controller_mode: u32,
    pub motion_generator_mode: u32,
    pub maximum_path_deviation_translation: f64,
    pub maximum_path_deviation_rotation: f64,
    pub maximum_path_deviation_elbow: f64,
    pub maximum_goal_pose_deviation_translation: f64,
    pub maximum_goal_pose_deviation_rotation: f64,
    pub maximum_goal_pose_deviation_elbow: f64,
    pub use_async_motion_generator: u8,
    pub maximum_velocity: [f64; 7],
}

/// Move command status codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MoveStatus {
    Success = 0,
    MotionStarted = 1,
    Preempted = 2,
    PreemptedDueToActivatedSafetyFunctions = 3,
    CommandRejectedDueToActivatedSafetyFunctions = 4,
    CommandNotPossibleRejected = 5,
    StartAtSingularPoseRejected = 6,
    InvalidArgumentRejected = 7,
    ReflexAborted = 8,
    EmergencyAborted = 9,
    InputErrorAborted = 10,
    Aborted = 11,
}

impl MoveStatus {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Success),
            1 => Some(Self::MotionStarted),
            2 => Some(Self::Preempted),
            3 => Some(Self::PreemptedDueToActivatedSafetyFunctions),
            4 => Some(Self::CommandRejectedDueToActivatedSafetyFunctions),
            5 => Some(Self::CommandNotPossibleRejected),
            6 => Some(Self::StartAtSingularPoseRejected),
            7 => Some(Self::InvalidArgumentRejected),
            8 => Some(Self::ReflexAborted),
            9 => Some(Self::EmergencyAborted),
            10 => Some(Self::InputErrorAborted),
            11 => Some(Self::Aborted),
            _ => None,
        }
    }
}

/// SetCollisionBehavior request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct SetCollisionBehaviorRequest {
    pub lower_torque_thresholds_acceleration: [f64; 7],
    pub upper_torque_thresholds_acceleration: [f64; 7],
    pub lower_torque_thresholds_nominal: [f64; 7],
    pub upper_torque_thresholds_nominal: [f64; 7],
    pub lower_force_thresholds_acceleration: [f64; 6],
    pub upper_force_thresholds_acceleration: [f64; 6],
    pub lower_force_thresholds_nominal: [f64; 6],
    pub upper_force_thresholds_nominal: [f64; 6],
}

/// SetJointImpedance request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct SetJointImpedanceRequest {
    pub k_theta: [f64; 7],
}

/// SetCartesianImpedance request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct SetCartesianImpedanceRequest {
    pub k_x: [f64; 6],
}

/// SetGuidingMode request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct SetGuidingModeRequest {
    pub guiding_mode: [u8; 6],
    pub nullspace: u8,
}

/// SetEEToK request (16-element column-major homogeneous transform).
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct SetEeToKRequest {
    pub ee_t_k: [f64; 16],
}

/// SetNEToEE request (16-element column-major homogeneous transform).
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct SetNeToEeRequest {
    pub ne_t_ee: [f64; 16],
}

/// SetLoad request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct SetLoadRequest {
    pub m_load: f64,
    pub f_x_cload: [f64; 3],
    pub i_load: [f64; 9],
}

/// Generic command response (status byte only).
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct CommandResponse {
    pub status: u8,
}

/// Generic getter/setter command status codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GetterSetterStatus {
    Success = 0,
    CommandNotPossibleRejected = 1,
    InvalidArgumentRejected = 2,
    CommandRejectedDueToActivatedSafetyFunctions = 3,
}

impl GetterSetterStatus {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Success),
            1 => Some(Self::CommandNotPossibleRejected),
            2 => Some(Self::InvalidArgumentRejected),
            3 => Some(Self::CommandRejectedDueToActivatedSafetyFunctions),
            _ => None,
        }
    }
}

/// StopMove status codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StopMoveStatus {
    Success = 0,
    CommandNotPossibleRejected = 1,
    CommandRejectedDueToActivatedSafetyFunctions = 2,
    EmergencyAborted = 3,
    ReflexAborted = 4,
    Aborted = 5,
}

/// AutomaticErrorRecovery status codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AutomaticErrorRecoveryStatus {
    Success = 0,
    CommandNotPossibleRejected = 1,
    CommandRejectedDueToActivatedSafetyFunctions = 2,
    ManualErrorRecoveryRequiredRejected = 3,
    ReflexAborted = 4,
    EmergencyAborted = 5,
    Aborted = 6,
}

// --- UDP wire types (robot state and commands) ---

/// Raw robot state as received over UDP.
///
/// This matches the C++ `research_interface::robot::RobotState` packed struct.
/// All multi-element fields use `f32` on the wire and are converted to `f64` in the public API.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct RawRobotState {
    pub message_id: u64,
    pub o_t_ee: [f32; 16],
    pub o_t_ee_d: [f32; 16],
    pub f_t_ee: [f32; 16],
    pub ee_t_k: [f32; 16],
    pub f_t_ne: [f32; 16],
    pub ne_t_ee: [f32; 16],
    pub m_ee: f32,
    pub i_ee: [f32; 9],
    pub f_x_cee: [f32; 3],
    pub m_load: f32,
    pub i_load: [f32; 9],
    pub f_x_cload: [f32; 3],
    pub elbow: [f32; 2],
    pub elbow_d: [f32; 2],
    pub tau_j: [f32; 7],
    pub tau_j_d: [f32; 7],
    pub dtau_j: [f32; 7],
    pub q: [f32; 7],
    pub q_d: [f32; 7],
    pub dq: [f32; 7],
    pub dq_d: [f32; 7],
    pub ddq_d: [f32; 7],
    pub joint_contact: [f32; 7],
    pub cartesian_contact: [f32; 6],
    pub joint_collision: [f32; 7],
    pub cartesian_collision: [f32; 6],
    pub tau_ext_hat_filtered: [f32; 7],
    pub o_f_ext_hat_k: [f32; 6],
    pub k_f_ext_hat_k: [f32; 6],
    pub o_dp_ee_d: [f32; 6],
    pub o_ddp_o: [f32; 3],
    pub elbow_c: [f32; 2],
    pub delbow_c: [f32; 2],
    pub ddelbow_c: [f32; 2],
    pub o_t_ee_c: [f32; 16],
    pub o_dp_ee_c: [f32; 6],
    pub o_ddp_ee_c: [f32; 6],
    pub theta: [f32; 7],
    pub dtheta: [f32; 7],
    /// 6 accelerometers x 3 axes (top PCB).
    pub accelerometer_top: [[f32; 3]; 6],
    /// 6 accelerometers x 3 axes (bottom PCB).
    pub accelerometer_bottom: [[f32; 3]; 6],
    pub motion_generator_mode: u8,
    pub controller_mode: u8,
    pub errors: [u8; 41],
    pub reflex_reason: [u8; 41],
    pub robot_mode: u8,
    pub control_command_success_rate: f32,
}

/// Motion generator command sent to the robot over UDP.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct MotionGeneratorCommand {
    pub q_c: [f64; 7],
    pub dq_c: [f64; 7],
    pub o_t_ee_c: [f64; 16],
    pub o_dp_ee_c: [f64; 6],
    pub elbow_c: [f64; 2],
    pub valid_elbow: u8,
    pub motion_generation_finished: u8,
}

/// Controller command sent to the robot over UDP.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct ControllerCommand {
    pub tau_j_d: [f64; 7],
    pub torque_command_finished: u8,
}

/// Full robot command sent over UDP each control cycle.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct RobotCommand {
    pub message_id: u64,
    pub motion: MotionGeneratorCommand,
    pub control: ControllerCommand,
}

// --- Byte conversion utilities for packed structs ---

impl CommandHeader {
    pub const SIZE: usize = std::mem::size_of::<Self>();

    /// # Safety
    /// `bytes` must be at least `SIZE` bytes long.
    pub unsafe fn from_bytes(bytes: &[u8]) -> Self {
        debug_assert!(bytes.len() >= Self::SIZE);
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const Self) }
    }

    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        unsafe { std::mem::transmute_copy(&self) }
    }
}

impl RawRobotState {
    pub const SIZE: usize = std::mem::size_of::<Self>();

    /// # Safety
    /// `bytes` must be at least `SIZE` bytes long.
    pub unsafe fn from_bytes(bytes: &[u8]) -> Self {
        debug_assert!(bytes.len() >= Self::SIZE);
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const Self) }
    }

    /// Convert wire-format errors (u8 array treated as bools) to a bool array.
    pub fn errors_as_bools(&self) -> [bool; 41] {
        let mut result = [false; 41];
        let errors = self.errors;
        for (i, &val) in errors.iter().enumerate() {
            result[i] = val != 0;
        }
        result
    }

    /// Convert wire-format reflex reasons to a bool array.
    pub fn reflex_reason_as_bools(&self) -> [bool; 41] {
        let mut result = [false; 41];
        let reflex = self.reflex_reason;
        for (i, &val) in reflex.iter().enumerate() {
            result[i] = val != 0;
        }
        result
    }

    /// Convert to the public RobotState type.
    pub fn to_robot_state(self) -> crate::robot_state::RobotState {
        use crate::errors::RobotErrors;
        use crate::robot_state::RobotState;
        use crate::types::{MotionGeneratorMode, RobotMode};

        // Copy fields out of packed struct to avoid unaligned references.
        let message_id = self.message_id;
        let motion_generator_mode = self.motion_generator_mode;
        let robot_mode = self.robot_mode;
        let control_command_success_rate = self.control_command_success_rate;

        RobotState {
            o_t_ee: f32x16_to_f64(self.o_t_ee),
            o_t_ee_d: f32x16_to_f64(self.o_t_ee_d),
            f_t_ee: f32x16_to_f64(self.f_t_ee),
            ee_t_k: f32x16_to_f64(self.ee_t_k),
            f_t_ne: f32x16_to_f64(self.f_t_ne),
            ne_t_ee: f32x16_to_f64(self.ne_t_ee),
            m_ee: self.m_ee as f64,
            i_ee: f32x9_to_f64(self.i_ee),
            f_x_cee: f32x3_to_f64(self.f_x_cee),
            m_load: self.m_load as f64,
            i_load: f32x9_to_f64(self.i_load),
            f_x_cload: f32x3_to_f64(self.f_x_cload),
            elbow: f32x2_to_f64(self.elbow),
            elbow_d: f32x2_to_f64(self.elbow_d),
            elbow_c: f32x2_to_f64(self.elbow_c),
            delbow_c: f32x2_to_f64(self.delbow_c),
            ddelbow_c: f32x2_to_f64(self.ddelbow_c),
            tau_j: f32x7_to_f64(self.tau_j),
            tau_j_d: f32x7_to_f64(self.tau_j_d),
            dtau_j: f32x7_to_f64(self.dtau_j),
            q: f32x7_to_f64(self.q),
            q_d: f32x7_to_f64(self.q_d),
            dq: f32x7_to_f64(self.dq),
            dq_d: f32x7_to_f64(self.dq_d),
            ddq_d: f32x7_to_f64(self.ddq_d),
            joint_contact: f32x7_to_f64(self.joint_contact),
            cartesian_contact: f32x6_to_f64(self.cartesian_contact),
            joint_collision: f32x7_to_f64(self.joint_collision),
            cartesian_collision: f32x6_to_f64(self.cartesian_collision),
            tau_ext_hat_filtered: f32x7_to_f64(self.tau_ext_hat_filtered),
            o_f_ext_hat_k: f32x6_to_f64(self.o_f_ext_hat_k),
            k_f_ext_hat_k: f32x6_to_f64(self.k_f_ext_hat_k),
            o_dp_ee_d: f32x6_to_f64(self.o_dp_ee_d),
            o_ddp_o: f32x3_to_f64(self.o_ddp_o),
            o_t_ee_c: f32x16_to_f64(self.o_t_ee_c),
            o_dp_ee_c: f32x6_to_f64(self.o_dp_ee_c),
            o_ddp_ee_c: f32x6_to_f64(self.o_ddp_ee_c),
            theta: f32x7_to_f64(self.theta),
            dtheta: f32x7_to_f64(self.dtheta),
            current_errors: RobotErrors::from_bool_array(&self.errors_as_bools()),
            last_motion_errors: RobotErrors::from_bool_array(&self.reflex_reason_as_bools()),
            control_command_success_rate: control_command_success_rate as f64,
            robot_mode: RobotMode::from_wire(robot_mode),
            motion_generator_mode: MotionGeneratorMode::from_wire(motion_generator_mode),
            time: std::time::Duration::from_millis(message_id),
        }
    }
}

impl RobotCommand {
    pub const SIZE: usize = std::mem::size_of::<Self>();

    pub fn to_bytes(self) -> Vec<u8> {
        let mut buf = vec![0u8; Self::SIZE];
        unsafe {
            std::ptr::write_unaligned(buf.as_mut_ptr() as *mut Self, self);
        }
        buf
    }
}

// Conversion helpers (concrete sizes to avoid generic issues with packed struct field copies).

fn f32x2_to_f64(src: [f32; 2]) -> [f64; 2] {
    [src[0] as f64, src[1] as f64]
}

fn f32x3_to_f64(src: [f32; 3]) -> [f64; 3] {
    [src[0] as f64, src[1] as f64, src[2] as f64]
}

fn f32x6_to_f64(src: [f32; 6]) -> [f64; 6] {
    let mut dst = [0.0f64; 6];
    for (d, s) in dst.iter_mut().zip(src.iter()) {
        *d = *s as f64;
    }
    dst
}

fn f32x7_to_f64(src: [f32; 7]) -> [f64; 7] {
    let mut dst = [0.0f64; 7];
    for (d, s) in dst.iter_mut().zip(src.iter()) {
        *d = *s as f64;
    }
    dst
}

fn f32x9_to_f64(src: [f32; 9]) -> [f64; 9] {
    let mut dst = [0.0f64; 9];
    for (d, s) in dst.iter_mut().zip(src.iter()) {
        *d = *s as f64;
    }
    dst
}

fn f32x16_to_f64(src: [f32; 16]) -> [f64; 16] {
    let mut dst = [0.0f64; 16];
    for (d, s) in dst.iter_mut().zip(src.iter()) {
        *d = *s as f64;
    }
    dst
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_header_size() {
        assert_eq!(CommandHeader::SIZE, 12);
    }

    #[test]
    fn connect_request_size() {
        assert_eq!(std::mem::size_of::<ConnectRequest>(), 4);
    }

    #[test]
    fn connect_response_size() {
        assert_eq!(std::mem::size_of::<ConnectResponse>(), 3);
    }

    #[test]
    fn motion_generator_command_size() {
        // 7*8 + 7*8 + 16*8 + 6*8 + 2*8 + 1 + 1 = 56+56+128+48+16+2 = 306
        assert_eq!(std::mem::size_of::<MotionGeneratorCommand>(), 306);
    }

    #[test]
    fn controller_command_size() {
        // 7*8 + 1 = 57
        assert_eq!(std::mem::size_of::<ControllerCommand>(), 57);
    }

    #[test]
    fn robot_command_size() {
        // 8 + 306 + 57 = 371
        assert_eq!(RobotCommand::SIZE, 371);
    }

    #[test]
    fn raw_robot_state_from_bytes_roundtrip() {
        let mut bytes = vec![0u8; RawRobotState::SIZE];
        bytes[0..8].copy_from_slice(&42u64.to_ne_bytes());

        let state = unsafe { RawRobotState::from_bytes(&bytes) };
        let msg_id = { state.message_id };
        assert_eq!(msg_id, 42);
    }

    #[test]
    fn robot_command_to_bytes_roundtrip() {
        let mut cmd: RobotCommand = unsafe { std::mem::zeroed() };
        cmd.message_id = 123;
        cmd.control.tau_j_d[0] = 1.5;

        let bytes = cmd.to_bytes();
        assert_eq!(bytes.len(), RobotCommand::SIZE);

        let recovered = unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const RobotCommand) };
        let msg_id = { recovered.message_id };
        let tau = { recovered.control.tau_j_d[0] };
        assert_eq!(msg_id, 123);
        assert_eq!(tau, 1.5);
    }

    #[test]
    fn errors_as_bools_conversion() {
        let mut state: RawRobotState = unsafe { std::mem::zeroed() };
        state.errors[0] = 1;
        state.errors[6] = 1;
        state.errors[40] = 1;

        let bools = state.errors_as_bools();
        assert!(bools[0]);
        assert!(!bools[1]);
        assert!(bools[6]);
        assert!(bools[40]);
    }

    #[test]
    fn f32_to_f64_conversion() {
        let src: [f32; 7] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
        let dst = f32x7_to_f64(src);
        assert_eq!(dst, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
    }
}
