/// Wire-format command enum for the gripper.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum Command {
    Connect = 0,
    Homing = 1,
    Grasp = 2,
    Move = 3,
    Stop = 4,
}

/// Gripper TCP message header.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct CommandHeader {
    pub command: u16,
    pub command_id: u32,
    pub size: u32,
}

/// Gripper connect request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct ConnectRequest {
    pub version: u16,
    pub udp_port: u16,
}

/// Gripper connect response.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct ConnectResponse {
    pub status: u16,
    pub version: u16,
}

/// Gripper grasp request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct GraspRequest {
    pub width: f64,
    pub epsilon_inner: f64,
    pub epsilon_outer: f64,
    pub speed: f64,
    pub force: f64,
}

/// Gripper move request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct MoveRequest {
    pub width: f64,
    pub speed: f64,
}

/// Generic gripper command response.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct CommandResponse {
    pub status: u16,
}

/// Gripper status codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum GripperStatus {
    Success = 0,
    Fail = 1,
    Unsuccessful = 2,
    Aborted = 3,
}

impl GripperStatus {
    pub fn from_u16(value: u16) -> Option<Self> {
        match value {
            0 => Some(Self::Success),
            1 => Some(Self::Fail),
            2 => Some(Self::Unsuccessful),
            3 => Some(Self::Aborted),
            _ => None,
        }
    }
}

/// Raw gripper state received over UDP.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct RawGripperState {
    pub message_id: u32,
    pub width: f64,
    pub max_width: f64,
    pub is_grasped: u8,
    pub temperature: u16,
}

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

impl RawGripperState {
    pub const SIZE: usize = std::mem::size_of::<Self>();

    /// # Safety
    /// `bytes` must be at least `SIZE` bytes long.
    pub unsafe fn from_bytes(bytes: &[u8]) -> Self {
        debug_assert!(bytes.len() >= Self::SIZE);
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const Self) }
    }
}
