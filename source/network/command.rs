use crate::errors::{FrankaError, FrankaResult};
use crate::wire::robot::{self, CommandHeader};

use super::Network;

/// High-level robot command interface.
///
/// Wraps the low-level Network TCP request/response into typed command execution.
pub struct RobotCommand<'a> {
    network: &'a mut Network,
}

impl<'a> RobotCommand<'a> {
    pub fn new(network: &'a mut Network) -> Self {
        Self { network }
    }

    /// Send a Move command to start a motion.
    pub fn start_motion(&mut self, request: &robot::MoveRequest) -> FrankaResult<u32> {
        let payload = struct_to_bytes(request);
        self.network
            .tcp_send_request(robot::Command::Move as u32, &payload)
    }

    /// Send a StopMove command.
    pub fn stop_motion(&mut self) -> FrankaResult<u32> {
        self.network
            .tcp_send_request(robot::Command::StopMove as u32, &[])
    }

    /// Set collision behavior thresholds.
    pub fn set_collision_behavior(
        &mut self,
        request: &robot::SetCollisionBehaviorRequest,
    ) -> FrankaResult<()> {
        self.execute_setter(robot::Command::SetCollisionBehavior as u32, request)
    }

    /// Set joint impedance parameters.
    pub fn set_joint_impedance(
        &mut self,
        request: &robot::SetJointImpedanceRequest,
    ) -> FrankaResult<()> {
        self.execute_setter(robot::Command::SetJointImpedance as u32, request)
    }

    /// Set Cartesian impedance parameters.
    pub fn set_cartesian_impedance(
        &mut self,
        request: &robot::SetCartesianImpedanceRequest,
    ) -> FrankaResult<()> {
        self.execute_setter(robot::Command::SetCartesianImpedance as u32, request)
    }

    /// Set guiding mode.
    pub fn set_guiding_mode(&mut self, request: &robot::SetGuidingModeRequest) -> FrankaResult<()> {
        self.execute_setter(robot::Command::SetGuidingMode as u32, request)
    }

    /// Set EE to K frame transform.
    pub fn set_ee_to_k(&mut self, request: &robot::SetEeToKRequest) -> FrankaResult<()> {
        self.execute_setter(robot::Command::SetEeToK as u32, request)
    }

    /// Set NE to EE frame transform.
    pub fn set_ne_to_ee(&mut self, request: &robot::SetNeToEeRequest) -> FrankaResult<()> {
        self.execute_setter(robot::Command::SetNeToEe as u32, request)
    }

    /// Set external load parameters.
    pub fn set_load(&mut self, request: &robot::SetLoadRequest) -> FrankaResult<()> {
        self.execute_setter(robot::Command::SetLoad as u32, request)
    }

    /// Trigger automatic error recovery.
    pub fn automatic_error_recovery(&mut self) -> FrankaResult<()> {
        let command_id = self
            .network
            .tcp_send_request(robot::Command::AutomaticErrorRecovery as u32, &[])?;
        let response_bytes = self.network.tcp_blocking_receive_response(command_id)?;
        let status = parse_response_status(&response_bytes)?;

        match status {
            0 => Ok(()),
            _ => Err(FrankaError::Command {
                message: format!(
                    "automatic error recovery failed with status {status}"
                ),
            }),
        }
    }

    /// Request the robot model URDF string.
    pub fn get_robot_model(&mut self) -> FrankaResult<String> {
        let command_id = self
            .network
            .tcp_send_request(robot::Command::GetRobotModel as u32, &[])?;
        let response_bytes = self.network.tcp_blocking_receive_response(command_id)?;

        // GetRobotModel response: header (12) + status (1) + URDF string
        if response_bytes.len() <= CommandHeader::SIZE + 1 {
            return Err(FrankaError::Protocol {
                message: "GetRobotModel response too short".into(),
            });
        }

        let status = response_bytes[CommandHeader::SIZE];
        if status != 0 {
            return Err(FrankaError::Command {
                message: format!("GetRobotModel failed with status {status}"),
            });
        }

        let urdf_bytes = &response_bytes[CommandHeader::SIZE + 1..];
        let urdf = String::from_utf8(urdf_bytes.to_vec()).map_err(|e| FrankaError::Protocol {
            message: format!("invalid URDF encoding: {e}"),
        })?;

        Ok(urdf)
    }

    /// Try to receive the response for a previously sent Move command (non-blocking).
    /// Returns Some(status) if a response is available.
    pub fn try_receive_move_response(&mut self, command_id: u32) -> FrankaResult<Option<u8>> {
        match self.network.tcp_try_receive_response(command_id)? {
            Some(response_bytes) => {
                let status = parse_response_status(&response_bytes)?;
                Ok(Some(status))
            }
            None => Ok(None),
        }
    }

    // --- Internal helpers ---

    /// Execute a setter command (send request, wait for response, check status).
    fn execute_setter<T: Copy>(&mut self, command: u32, request: &T) -> FrankaResult<()> {
        let payload = struct_to_bytes(request);
        let command_id = self.network.tcp_send_request(command, &payload)?;
        let response_bytes = self.network.tcp_blocking_receive_response(command_id)?;
        let status = parse_response_status(&response_bytes)?;

        match robot::GetterSetterStatus::from_u8(status) {
            Some(robot::GetterSetterStatus::Success) => Ok(()),
            Some(robot::GetterSetterStatus::CommandNotPossibleRejected) => {
                Err(FrankaError::Command {
                    message: "command not possible in current state".into(),
                })
            }
            Some(robot::GetterSetterStatus::InvalidArgumentRejected) => {
                Err(FrankaError::Command {
                    message: "invalid argument".into(),
                })
            }
            Some(robot::GetterSetterStatus::CommandRejectedDueToActivatedSafetyFunctions) => {
                Err(FrankaError::Command {
                    message: "command rejected due to activated safety functions".into(),
                })
            }
            None => Err(FrankaError::Protocol {
                message: format!("unknown setter status: {status}"),
            }),
        }
    }
}

/// Extract the status byte from a response message.
fn parse_response_status(response_bytes: &[u8]) -> FrankaResult<u8> {
    if response_bytes.len() <= CommandHeader::SIZE {
        return Err(FrankaError::Protocol {
            message: "response too short to contain status".into(),
        });
    }
    Ok(response_bytes[CommandHeader::SIZE])
}

/// Convert a packed struct to bytes for sending.
fn struct_to_bytes<T: Copy>(value: &T) -> Vec<u8> {
    let size = std::mem::size_of::<T>();
    let mut bytes = vec![0u8; size];
    unsafe {
        std::ptr::copy_nonoverlapping(value as *const T as *const u8, bytes.as_mut_ptr(), size);
    }
    bytes
}
