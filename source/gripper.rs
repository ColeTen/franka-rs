use crate::constants::GRIPPER_COMMAND_PORT;
use crate::errors::{FrankaError, FrankaResult};
use crate::network::{self, Network, NetworkConfig};
use crate::wire::gripper::{self, GripperStatus, RawGripperState};

/// Current state of the Franka gripper.
#[derive(Debug, Clone)]
pub struct GripperState {
    /// Current gripper opening width in meters.
    pub width: f64,
    /// Maximum gripper opening width (estimated by homing) in meters.
    pub max_width: f64,
    /// Whether an object is currently grasped.
    pub is_grasped: bool,
    /// Current gripper temperature in degrees Celsius.
    pub temperature: u16,
}

/// Interface for the Franka parallel gripper.
///
/// Connects to the gripper on port 1338 (same IP as the robot).
pub struct Gripper {
    network: Network<gripper::CommandHeader>,
    server_version: u16,
}

impl Gripper {
    /// Connect to the gripper at the given robot address.
    pub fn connect(address: &str) -> FrankaResult<Self> {
        let config = NetworkConfig::default();
        let mut network = Network::connect_with_header(address, GRIPPER_COMMAND_PORT, &config)?;
        let server_version = network::connect_gripper(&mut network)?;

        Ok(Self {
            network,
            server_version,
        })
    }

    /// Returns the server protocol version.
    pub fn server_version(&self) -> u16 {
        self.server_version
    }

    /// Perform homing to calibrate the gripper.
    ///
    /// Must be done after changing gripper fingers to estimate max grasping width.
    pub fn homing(&mut self) -> FrankaResult<bool> {
        self.execute_command(gripper::Command::Homing, &[])
    }

    /// Grasp an object.
    ///
    /// An object is considered grasped if the finger distance `d` satisfies:
    /// `(width - epsilon_inner) < d < (width + epsilon_outer)`
    ///
    /// Returns `true` if the object was successfully grasped.
    pub fn grasp(
        &mut self,
        width: f64,
        speed: f64,
        force: f64,
        epsilon_inner: f64,
        epsilon_outer: f64,
    ) -> FrankaResult<bool> {
        let request = gripper::GraspRequest {
            width,
            epsilon_inner,
            epsilon_outer,
            speed,
            force,
        };
        let payload = struct_to_bytes(&request);
        self.execute_command(gripper::Command::Grasp, &payload)
    }

    /// Move the gripper fingers to the specified width.
    ///
    /// Returns `true` if the move completed successfully.
    pub fn move_fingers(&mut self, width: f64, speed: f64) -> FrankaResult<bool> {
        let request = gripper::MoveRequest { width, speed };
        let payload = struct_to_bytes(&request);
        self.execute_command(gripper::Command::Move, &payload)
    }

    /// Stop a currently running gripper move or grasp.
    pub fn stop(&mut self) -> FrankaResult<bool> {
        self.execute_command(gripper::Command::Stop, &[])
    }

    /// Read the current gripper state.
    pub fn read_once(&self) -> FrankaResult<GripperState> {
        let mut buf = [0u8; RawGripperState::SIZE + 64];
        let n = self.network.udp_blocking_receive(&mut buf)?;

        if n < RawGripperState::SIZE {
            return Err(FrankaError::Protocol {
                message: format!(
                    "gripper UDP state too small: {n} < {}",
                    RawGripperState::SIZE
                ),
            });
        }

        let raw = unsafe { RawGripperState::from_bytes(&buf[..n]) };
        let width = { raw.width };
        let max_width = { raw.max_width };
        let is_grasped = { raw.is_grasped };
        let temperature = { raw.temperature };

        Ok(GripperState {
            width,
            max_width,
            is_grasped: is_grasped != 0,
            temperature,
        })
    }

    /// Send a command and wait for the response, returning success/failure.
    fn execute_command(
        &mut self,
        command: gripper::Command,
        payload: &[u8],
    ) -> FrankaResult<bool> {
        let command_id = self
            .network
            .tcp_send_request(command as u16, payload)?;
        let response_bytes = self.network.tcp_blocking_receive_response(command_id)?;

        let status = parse_gripper_status(&response_bytes)?;
        match status {
            GripperStatus::Success => Ok(true),
            GripperStatus::Unsuccessful => Ok(false),
            GripperStatus::Fail => Err(FrankaError::Command {
                message: format!("gripper command {command:?} failed"),
            }),
            GripperStatus::Aborted => Err(FrankaError::Command {
                message: format!("gripper command {command:?} aborted"),
            }),
        }
    }
}

fn parse_gripper_status(response_bytes: &[u8]) -> FrankaResult<GripperStatus> {
    if response_bytes.len() < gripper::CommandHeader::SIZE + 2 {
        return Err(FrankaError::Protocol {
            message: "gripper response too short".into(),
        });
    }

    let status_bytes = &response_bytes[gripper::CommandHeader::SIZE..];
    let status_u16 = u16::from_ne_bytes([status_bytes[0], status_bytes[1]]);

    GripperStatus::from_u16(status_u16).ok_or_else(|| FrankaError::Protocol {
        message: format!("unknown gripper status: {status_u16}"),
    })
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
    use super::*;
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::thread;

    /// Encodes a gripper message: the 10-byte header followed by `payload`.
    fn message(command: u16, command_id: u32, payload: &[u8]) -> Vec<u8> {
        let size = (gripper::CommandHeader::SIZE + payload.len()) as u32;
        let mut bytes = gripper::CommandHeader { command, command_id, size }.to_bytes().to_vec();
        bytes.extend_from_slice(payload);
        bytes
    }

    /// Reads exactly `length` bytes from `stream`.
    fn read_exact(stream: &mut impl Read, length: usize) -> Vec<u8> {
        let mut bytes = vec![0u8; length];
        stream.read_exact(&mut bytes).unwrap();
        bytes
    }

    #[test]
    fn connect_and_homing_use_the_gripper_header() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let connect_request = read_exact(&mut stream, 14);
            stream.write_all(&message(0, 0, &[0, 0, 3, 0])).unwrap(); // Success, version 3
            let homing_request = read_exact(&mut stream, 10);
            stream.write_all(&message(1, 1, &[0, 0])).unwrap(); // Success
            (connect_request, homing_request)
        });

        let mut network =
            Network::<gripper::CommandHeader>::connect_with_header("127.0.0.1", port, &NetworkConfig::default())
                .unwrap();
        let udp_port = network.udp_port();
        let server_version = network::connect_gripper(&mut network).unwrap();
        let mut gripper = Gripper { network, server_version };
        assert_eq!(gripper.server_version(), crate::constants::GRIPPER_PROTOCOL_VERSION);
        assert!(gripper.homing().unwrap());

        let (connect_request, homing_request) = server.join().unwrap();
        let mut connect_payload = crate::constants::GRIPPER_PROTOCOL_VERSION.to_ne_bytes().to_vec();
        connect_payload.extend_from_slice(&udp_port.to_ne_bytes());
        assert_eq!(connect_request, message(0, 0, &connect_payload));
        assert_eq!(homing_request, message(1, 1, &[]));
    }

    #[test]
    fn status_is_read_after_the_gripper_header() {
        assert_eq!(parse_gripper_status(&message(3, 0, &[2, 0])).unwrap(), GripperStatus::Unsuccessful);
        assert!(parse_gripper_status(&message(3, 0, &[])).is_err());
    }
}
