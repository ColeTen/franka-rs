use crate::control_loop;
use crate::control_types::MotionType;
use crate::errors::{FrankaError, FrankaResult};
use crate::network::Network;
use crate::robot_state::RobotState;
use crate::types::{ControllerMode, MotionGeneratorMode, Torques};
use crate::wire::robot::{ControllerCommand, MotionGeneratorCommand, RawRobotState, RobotCommand};

/// Active torque control session — read state and write torques without a callback.
///
/// Created via `Robot::start_torque_control()`. The motion is started on creation
/// and finalized on drop.
pub struct ActiveTorqueControl<'a> {
    network: &'a mut Network,
    motion_id: u32,
    message_id: u64,
    finished: bool,
}

impl<'a> ActiveTorqueControl<'a> {
    pub(crate) fn start(network: &'a mut Network) -> FrankaResult<Self> {
        let motion_id = control_loop::start_motion(
            network,
            ControllerMode::ExternalController,
            MotionGeneratorMode::None,
        )?;

        Ok(Self {
            network,
            motion_id,
            message_id: 0,
            finished: false,
        })
    }

    /// Read the latest robot state from the robot.
    pub fn read_state(&self) -> FrankaResult<RobotState> {
        let mut buf = [0u8; RawRobotState::SIZE + 128];
        let n = self.network.udp_blocking_receive(&mut buf)?;

        if n < RawRobotState::SIZE {
            return Err(FrankaError::Protocol {
                message: format!(
                    "UDP state too small: {n} < {}",
                    RawRobotState::SIZE
                ),
            });
        }

        let raw = unsafe { RawRobotState::from_bytes(&buf[..n]) };
        Ok(raw.to_robot_state())
    }

    /// Send torque commands to the robot.
    ///
    /// Returns the robot state received after sending.
    pub fn write_torques(&mut self, torques: &Torques) -> FrankaResult<RobotState> {
        let tau_j_d: [f64; 7] = **torques;

        let control_cmd = ControllerCommand {
            tau_j_d,
            torque_command_finished: 0,
        };

        let motion_cmd = MotionGeneratorCommand {
            q_c: [0.0; 7],
            dq_c: [0.0; 7],
            o_t_ee_c: [0.0; 16],
            o_dp_ee_c: [0.0; 6],
            elbow_c: [0.0; 2],
            valid_elbow: 0,
            motion_generation_finished: 0,
        };

        let robot_cmd = RobotCommand {
            message_id: self.message_id,
            motion: motion_cmd,
            control: control_cmd,
        };

        self.message_id += 1;
        let bytes = struct_to_bytes(&robot_cmd);
        self.network.udp_send(&bytes)?;

        self.read_state()
    }

    /// Signal that this control session is finished.
    ///
    /// Sends the final command and releases control. Called automatically on drop.
    pub fn finish(&mut self) -> FrankaResult<()> {
        if self.finished {
            return Ok(());
        }
        self.finished = true;

        let control_cmd = ControllerCommand {
            tau_j_d: [0.0; 7],
            torque_command_finished: 1,
        };

        let motion_cmd = MotionGeneratorCommand {
            q_c: [0.0; 7],
            dq_c: [0.0; 7],
            o_t_ee_c: [0.0; 16],
            o_dp_ee_c: [0.0; 6],
            elbow_c: [0.0; 2],
            valid_elbow: 0,
            motion_generation_finished: 0,
        };

        let robot_cmd = RobotCommand {
            message_id: self.message_id,
            motion: motion_cmd,
            control: control_cmd,
        };

        let bytes = struct_to_bytes(&robot_cmd);
        self.network.udp_send(&bytes)?;

        control_loop::finish_motion(self.network, self.motion_id)
    }
}

impl Drop for ActiveTorqueControl<'_> {
    fn drop(&mut self) {
        if !self.finished {
            let _ = self.finish();
        }
    }
}

/// Active motion control session — read state and write motion commands without a callback.
///
/// Created via `Robot::start_motion_control::<M>()`. The motion is started on creation
/// and finalized on drop.
pub struct ActiveMotionControl<'a, M: MotionType> {
    network: &'a mut Network,
    motion_id: u32,
    message_id: u64,
    finished: bool,
    _marker: std::marker::PhantomData<M>,
}

impl<'a, M: MotionType> ActiveMotionControl<'a, M> {
    pub(crate) fn start(
        network: &'a mut Network,
        controller_mode: ControllerMode,
    ) -> FrankaResult<Self> {
        let motion_id = control_loop::start_motion(
            network,
            controller_mode,
            M::motion_generator_mode(),
        )?;

        Ok(Self {
            network,
            motion_id,
            message_id: 0,
            finished: false,
            _marker: std::marker::PhantomData,
        })
    }

    /// Read the latest robot state.
    pub fn read_state(&self) -> FrankaResult<RobotState> {
        let mut buf = [0u8; RawRobotState::SIZE + 128];
        let n = self.network.udp_blocking_receive(&mut buf)?;

        if n < RawRobotState::SIZE {
            return Err(FrankaError::Protocol {
                message: format!(
                    "UDP state too small: {n} < {}",
                    RawRobotState::SIZE
                ),
            });
        }

        let raw = unsafe { RawRobotState::from_bytes(&buf[..n]) };
        Ok(raw.to_robot_state())
    }

    /// Send a motion command to the robot.
    ///
    /// Returns the robot state received after sending.
    pub fn write_motion(&mut self, command: &MotionGeneratorCommand) -> FrankaResult<RobotState> {
        let control_cmd = ControllerCommand {
            tau_j_d: [0.0; 7],
            torque_command_finished: 0,
        };

        let robot_cmd = RobotCommand {
            message_id: self.message_id,
            motion: *command,
            control: control_cmd,
        };

        self.message_id += 1;
        let bytes = struct_to_bytes(&robot_cmd);
        self.network.udp_send(&bytes)?;

        self.read_state()
    }

    /// Send both motion and torque commands simultaneously.
    pub fn write_motion_with_torques(
        &mut self,
        motion: &MotionGeneratorCommand,
        torques: &Torques,
    ) -> FrankaResult<RobotState> {
        let tau_j_d: [f64; 7] = **torques;
        let control_cmd = ControllerCommand {
            tau_j_d,
            torque_command_finished: 0,
        };

        let robot_cmd = RobotCommand {
            message_id: self.message_id,
            motion: *motion,
            control: control_cmd,
        };

        self.message_id += 1;
        let bytes = struct_to_bytes(&robot_cmd);
        self.network.udp_send(&bytes)?;

        self.read_state()
    }

    /// Signal that this motion is finished.
    pub fn finish(&mut self) -> FrankaResult<()> {
        if self.finished {
            return Ok(());
        }
        self.finished = true;

        let motion_cmd = MotionGeneratorCommand {
            q_c: [0.0; 7],
            dq_c: [0.0; 7],
            o_t_ee_c: [0.0; 16],
            o_dp_ee_c: [0.0; 6],
            elbow_c: [0.0; 2],
            valid_elbow: 0,
            motion_generation_finished: 1,
        };

        let control_cmd = ControllerCommand {
            tau_j_d: [0.0; 7],
            torque_command_finished: 0,
        };

        let robot_cmd = RobotCommand {
            message_id: self.message_id,
            motion: motion_cmd,
            control: control_cmd,
        };

        let bytes = struct_to_bytes(&robot_cmd);
        self.network.udp_send(&bytes)?;

        control_loop::finish_motion(self.network, self.motion_id)
    }
}

impl<M: MotionType> Drop for ActiveMotionControl<'_, M> {
    fn drop(&mut self) {
        if !self.finished {
            let _ = self.finish();
        }
    }
}

fn struct_to_bytes<T: Copy>(value: &T) -> Vec<u8> {
    let size = std::mem::size_of::<T>();
    let mut bytes = vec![0u8; size];
    unsafe {
        std::ptr::copy_nonoverlapping(value as *const T as *const u8, bytes.as_mut_ptr(), size);
    }
    bytes
}
