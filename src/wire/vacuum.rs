/// Wire-format command enum for the vacuum gripper.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum Command {
    Connect = 0,
    Vacuum = 1,
    DropOff = 2,
    Stop = 3,
}

/// Wire-format vacuum profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Profile {
    P0 = 0,
    P1 = 1,
    P2 = 2,
    P3 = 3,
}

/// Wire-format device status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DeviceStatus {
    Green = 0,
    Yellow = 1,
    Orange = 2,
    Red = 3,
}

impl DeviceStatus {
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Green),
            1 => Some(Self::Yellow),
            2 => Some(Self::Orange),
            3 => Some(Self::Red),
            _ => None,
        }
    }
}

/// Vacuum gripper TCP message header.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct CommandHeader {
    pub command: u16,
    pub command_id: u32,
    pub size: u32,
}

/// Vacuum gripper connect request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct ConnectRequest {
    pub version: u16,
    pub udp_port: u16,
}

/// Vacuum gripper connect response.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct ConnectResponse {
    pub status: u16,
    pub version: u16,
}

/// Vacuum command request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct VacuumRequest {
    pub vacuum: u8,
    pub profile: u8,
    pub timeout_ms: u64,
}

/// DropOff command request.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct DropOffRequest {
    pub timeout_ms: u64,
}

/// Generic vacuum gripper command response.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct CommandResponse {
    pub status: u16,
}

/// Vacuum gripper status codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum VacuumGripperStatus {
    Success = 0,
    Fail = 1,
    Unsuccessful = 2,
    Aborted = 3,
}

impl VacuumGripperStatus {
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

/// Raw vacuum gripper state received over UDP.
#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
pub struct RawVacuumGripperState {
    pub message_id: u32,
    pub in_control_range: u8,
    pub part_detached: u8,
    pub part_present: u8,
    pub device_status: u8,
    pub actual_power: i32,
    pub vacuum: i32,
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

impl RawVacuumGripperState {
    pub const SIZE: usize = std::mem::size_of::<Self>();

    /// # Safety
    /// `bytes` must be at least `SIZE` bytes long.
    pub unsafe fn from_bytes(bytes: &[u8]) -> Self {
        debug_assert!(bytes.len() >= Self::SIZE);
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const Self) }
    }
}
