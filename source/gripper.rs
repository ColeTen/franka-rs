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
    network: Network,
    server_version: u16,
}

impl Gripper {
    /// Connect to the gripper at the given robot address.
    pub fn connect(address: &str) -> FrankaResult<Self> {
        let config = NetworkConfig::default();
        let mut network = Network::connect(address, GRIPPER_COMMAND_PORT, &config)?;
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
            .tcp_send_request(command as u16 as u32, payload)?;
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
