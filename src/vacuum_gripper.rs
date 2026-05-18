use std::time::Duration;

use crate::constants::VACUUM_GRIPPER_COMMAND_PORT;
use crate::errors::{FrankaError, FrankaResult};
use crate::network::{self, Network, NetworkConfig};
use crate::wire::vacuum::{self, DeviceStatus, RawVacuumGripperState, VacuumGripperStatus};

/// Production setup profile for the vacuum gripper.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VacuumProfile {
    P0,
    P1,
    P2,
    P3,
}

impl VacuumProfile {
    fn to_wire(self) -> u8 {
        match self {
            Self::P0 => 0,
            Self::P1 => 1,
            Self::P2 => 2,
            Self::P3 => 3,
        }
    }
}

/// Device status of the vacuum gripper.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VacuumDeviceStatus {
    /// Device is working optimally.
    Green,
    /// Device is working but there are warnings.
    Yellow,
    /// Device is working but there are severe warnings.
    Orange,
    /// Device is not working properly.
    Red,
}

/// Current state of the vacuum gripper.
#[derive(Debug, Clone)]
pub struct VacuumGripperState {
    /// Vacuum value is within the setpoint area.
    pub in_control_range: bool,
    /// The part has been detached after a suction cycle.
    pub part_detached: bool,
    /// Vacuum is over H2 threshold (part present).
    pub part_present: bool,
    /// Current device status.
    pub device_status: VacuumDeviceStatus,
    /// Current actual power in percent.
    pub actual_power: u16,
    /// Current system vacuum in mbar.
    pub vacuum: u16,
}

/// Interface for the Franka vacuum gripper (cobot pump).
///
/// Connects on port 1339 (same IP as the robot).
pub struct VacuumGripper {
    network: Network,
    server_version: u16,
}

impl VacuumGripper {
    /// Connect to the vacuum gripper at the given robot address.
    pub fn connect(address: &str) -> FrankaResult<Self> {
        let config = NetworkConfig::default();
        let mut network = Network::connect(address, VACUUM_GRIPPER_COMMAND_PORT, &config)?;
        let server_version = network::connect_vacuum_gripper(&mut network)?;

        Ok(Self {
            network,
            server_version,
        })
    }

    /// Returns the server protocol version.
    pub fn server_version(&self) -> u16 {
        self.server_version
    }

    /// Activate vacuum to grasp an object.
    ///
    /// * `vacuum_setpoint` — Target vacuum in units of 10*mbar.
    /// * `timeout` — Maximum time to wait for vacuum to establish.
    /// * `profile` — Production setup profile (P0-P3).
    ///
    /// Returns `true` if vacuum was established within the timeout.
    pub fn vacuum(
        &mut self,
        vacuum_setpoint: u8,
        timeout: Duration,
        profile: VacuumProfile,
    ) -> FrankaResult<bool> {
        let request = vacuum::VacuumRequest {
            vacuum: vacuum_setpoint,
            profile: profile.to_wire(),
            timeout_ms: timeout.as_millis() as u64,
        };
        let payload = struct_to_bytes(&request);
        self.execute_command(vacuum::Command::Vacuum, &payload)
    }

    /// Drop off the grasped object (release vacuum).
    ///
    /// * `timeout` — Maximum time to wait for dropoff.
    ///
    /// Returns `true` if the dropoff completed within the timeout.
    pub fn drop_off(&mut self, timeout: Duration) -> FrankaResult<bool> {
        let request = vacuum::DropOffRequest {
            timeout_ms: timeout.as_millis() as u64,
        };
        let payload = struct_to_bytes(&request);
        self.execute_command(vacuum::Command::DropOff, &payload)
    }

    /// Stop a currently running vacuum or drop off operation.
    pub fn stop(&mut self) -> FrankaResult<bool> {
        self.execute_command(vacuum::Command::Stop, &[])
    }

    /// Read the current vacuum gripper state.
    pub fn read_once(&self) -> FrankaResult<VacuumGripperState> {
        let mut buf = [0u8; RawVacuumGripperState::SIZE + 64];
        let n = self.network.udp_blocking_receive(&mut buf)?;

        if n < RawVacuumGripperState::SIZE {
            return Err(FrankaError::Protocol {
                message: format!(
                    "vacuum gripper UDP state too small: {n} < {}",
                    RawVacuumGripperState::SIZE
                ),
            });
        }

        let raw = unsafe { RawVacuumGripperState::from_bytes(&buf[..n]) };
        let in_control_range = { raw.in_control_range };
        let part_detached = { raw.part_detached };
        let part_present = { raw.part_present };
        let device_status_raw = { raw.device_status };
        let actual_power = { raw.actual_power };
        let vacuum_val = { raw.vacuum };

        let device_status = match DeviceStatus::from_u8(device_status_raw) {
            Some(DeviceStatus::Green) => VacuumDeviceStatus::Green,
            Some(DeviceStatus::Yellow) => VacuumDeviceStatus::Yellow,
            Some(DeviceStatus::Orange) => VacuumDeviceStatus::Orange,
            Some(DeviceStatus::Red) => VacuumDeviceStatus::Red,
            None => VacuumDeviceStatus::Red,
        };

        Ok(VacuumGripperState {
            in_control_range: in_control_range != 0,
            part_detached: part_detached != 0,
            part_present: part_present != 0,
            device_status,
            actual_power: actual_power as u16,
            vacuum: vacuum_val as u16,
        })
    }

    fn execute_command(
        &mut self,
        command: vacuum::Command,
        payload: &[u8],
    ) -> FrankaResult<bool> {
        let command_id = self
            .network
            .tcp_send_request(command as u16 as u32, payload)?;
        let response_bytes = self.network.tcp_blocking_receive_response(command_id)?;

        let status = parse_vacuum_status(&response_bytes)?;
        match status {
            VacuumGripperStatus::Success => Ok(true),
            VacuumGripperStatus::Unsuccessful => Ok(false),
            VacuumGripperStatus::Fail => Err(FrankaError::Command {
                message: format!("vacuum gripper command {command:?} failed"),
            }),
            VacuumGripperStatus::Aborted => Err(FrankaError::Command {
                message: format!("vacuum gripper command {command:?} aborted"),
            }),
        }
    }
}

fn parse_vacuum_status(response_bytes: &[u8]) -> FrankaResult<VacuumGripperStatus> {
    if response_bytes.len() < vacuum::CommandHeader::SIZE + 2 {
        return Err(FrankaError::Protocol {
            message: "vacuum gripper response too short".into(),
        });
    }

    let status_bytes = &response_bytes[vacuum::CommandHeader::SIZE..];
    let status_u16 = u16::from_ne_bytes([status_bytes[0], status_bytes[1]]);

    VacuumGripperStatus::from_u16(status_u16).ok_or_else(|| FrankaError::Protocol {
        message: format!("unknown vacuum gripper status: {status_u16}"),
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
