use crate::constants;
use crate::errors::{FrankaError, FrankaResult};
use crate::wire::robot::{self, CommandHeader, ConnectRequest, ConnectResponse};

use super::Network;

/// Perform the robot connection handshake.
///
/// Sends a Connect request with the library protocol version and UDP port,
/// then waits for the server's response. Returns the server's protocol version
/// on success.
pub fn connect_robot(network: &mut Network) -> FrankaResult<u16> {
    let request = ConnectRequest {
        version: constants::ROBOT_PROTOCOL_VERSION,
        udp_port: network.udp_port(),
    };

    let payload = request_to_bytes(&request);
    let command_id =
        network.tcp_send_request(robot::Command::Connect as u32, &payload)?;

    let response_bytes = network.tcp_blocking_receive_response(command_id)?;

    // Response is: header (12 bytes) + ConnectResponse
    if response_bytes.len() < CommandHeader::SIZE + std::mem::size_of::<ConnectResponse>() {
        return Err(FrankaError::Protocol {
            message: "Connect response too short".into(),
        });
    }

    let response = unsafe {
        ConnectResponse::from_bytes(&response_bytes[CommandHeader::SIZE..])
    };

    let status = { response.status };
    let version = { response.version };

    match status {
        0 => Ok(version), // Success
        1 => Err(FrankaError::IncompatibleVersion {
            server_version: version,
            library_version: constants::ROBOT_PROTOCOL_VERSION,
        }),
        _ => Err(FrankaError::Protocol {
            message: format!("unexpected connect status: {status}"),
        }),
    }
}

/// Perform the gripper connection handshake.
pub fn connect_gripper(network: &mut Network) -> FrankaResult<u16> {
    use crate::wire::gripper;

    let request = gripper::ConnectRequest {
        version: constants::GRIPPER_PROTOCOL_VERSION,
        udp_port: network.udp_port(),
    };

    let payload = request_to_bytes(&request);
    let command_id =
        network.tcp_send_request(gripper::Command::Connect as u16 as u32, &payload)?;

    let response_bytes = network.tcp_blocking_receive_response(command_id)?;

    if response_bytes.len()
        < gripper::CommandHeader::SIZE + std::mem::size_of::<gripper::ConnectResponse>()
    {
        return Err(FrankaError::Protocol {
            message: "Gripper connect response too short".into(),
        });
    }

    let response = unsafe {
        gripper::ConnectResponse::from_bytes(&response_bytes[gripper::CommandHeader::SIZE..])
    };

    let status = { response.status };
    let version = { response.version };

    match status {
        0 => Ok(version),
        1 => Err(FrankaError::IncompatibleVersion {
            server_version: version,
            library_version: constants::GRIPPER_PROTOCOL_VERSION,
        }),
        _ => Err(FrankaError::Protocol {
            message: format!("unexpected gripper connect status: {status}"),
        }),
    }
}

/// Perform the vacuum gripper connection handshake.
pub fn connect_vacuum_gripper(network: &mut Network) -> FrankaResult<u16> {
    use crate::wire::vacuum;

    let request = vacuum::ConnectRequest {
        version: constants::VACUUM_GRIPPER_PROTOCOL_VERSION,
        udp_port: network.udp_port(),
    };

    let payload = request_to_bytes(&request);
    let command_id =
        network.tcp_send_request(vacuum::Command::Connect as u16 as u32, &payload)?;

    let response_bytes = network.tcp_blocking_receive_response(command_id)?;

    if response_bytes.len()
        < vacuum::CommandHeader::SIZE + std::mem::size_of::<vacuum::ConnectResponse>()
    {
        return Err(FrankaError::Protocol {
            message: "Vacuum gripper connect response too short".into(),
        });
    }

    let response = unsafe {
        vacuum::ConnectResponse::from_bytes(&response_bytes[vacuum::CommandHeader::SIZE..])
    };

    let status = { response.status };
    let version = { response.version };

    match status {
        0 => Ok(version),
        1 => Err(FrankaError::IncompatibleVersion {
            server_version: version,
            library_version: constants::VACUUM_GRIPPER_PROTOCOL_VERSION,
        }),
        _ => Err(FrankaError::Protocol {
            message: format!("unexpected vacuum gripper connect status: {status}"),
        }),
    }
}

/// Convert a packed struct to a byte slice for transmission.
fn request_to_bytes<T: Copy>(request: &T) -> Vec<u8> {
    let size = std::mem::size_of::<T>();
    let mut bytes = vec![0u8; size];
    unsafe {
        std::ptr::copy_nonoverlapping(request as *const T as *const u8, bytes.as_mut_ptr(), size);
    }
    bytes
}

// Add from_bytes to ConnectResponse (it doesn't have one yet that accepts a slice)
impl ConnectResponse {
    /// # Safety
    /// `bytes` must be at least `size_of::<Self>()` bytes long.
    pub(crate) unsafe fn from_bytes(bytes: &[u8]) -> Self {
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const Self) }
    }
}

impl crate::wire::gripper::ConnectResponse {
    /// # Safety
    /// `bytes` must be at least `size_of::<Self>()` bytes long.
    pub(crate) unsafe fn from_bytes(bytes: &[u8]) -> Self {
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const Self) }
    }
}

impl crate::wire::vacuum::ConnectResponse {
    /// # Safety
    /// `bytes` must be at least `size_of::<Self>()` bytes long.
    pub(crate) unsafe fn from_bytes(bytes: &[u8]) -> Self {
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const Self) }
    }
}
