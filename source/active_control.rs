use crate::control_loop;
use crate::control_types::MotionType;
use crate::errors::{FrankaError, FrankaResult};
use crate::network::Network;
use crate::robot_state::RobotState;
use crate::types::{ControllerMode, MotionGeneratorMode, Torques};
use crate::wire::robot::{ControllerCommand, RobotCommand};

/// Active torque control session — read state and write torques without a callback.
///
/// Created via `Robot::start_torque_control()`. The motion is started on creation. End it with
/// [`ActiveTorqueControl::finish`]; dropping it unfinished cancels the motion (StopMove), as
/// libfranka does.
pub struct ActiveTorqueControl<'a> {
    network: &'a mut Network,
    motion_id: u32,
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
            finished: false,
        })
    }

    /// Read the latest robot state from the robot.
    ///
    /// Returns an error once the robot has left the motion (for example after a user stop or a
    /// reflex), as libfranka's `readOnce` does, and, without reading, once the session has finished.
    pub fn read_state(&mut self) -> FrankaResult<RobotState> {
        if self.finished {
            return Err(finished_error());
        }
        let state = control_loop::receive_robot_state(self.network)?;
        control_loop::check_motion_error(&state, self.motion_id, self.network)?;
        Ok(state)
    }

    /// Send torque commands to the robot.
    ///
    /// Returns the robot state received after sending. Returns an error, without sending, if the
    /// session has finished or a torque is not finite.
    pub fn write_torques(&mut self, torques: &Torques) -> FrankaResult<RobotState> {
        if self.finished {
            return Err(finished_error());
        }
        let tau_j_d: [f64; 7] = **torques;
        crate::command_checks::check_finite(&tau_j_d)?;

        let control_cmd = ControllerCommand {
            tau_j_d,
            torque_command_finished: 0,
        };

        let robot_cmd = RobotCommand {
            message_id: self.network.latest_state_message_id(),
            motion: crate::motion_conversion::empty_command(),
            control: control_cmd,
        };
        control_loop::send_robot_command(self.network, &robot_cmd)?;

        self.read_state()
    }

    /// Ends the control session with `torques` as the final command, as libfranka's
    /// `writeOnce` with `motion_finished` set does: the command is sent with its finished flag
    /// while the robot still reports the controller running, then the Move response is awaited.
    ///
    /// Returns an error, without sending, if the session has already finished, a torque is not
    /// finite, or a motion generator is running. If finishing fails, the session stays unfinished
    /// and dropping it cancels the motion.
    pub fn finish(&mut self, torques: &Torques) -> FrankaResult<()> {
        if self.finished {
            return Err(finished_error());
        }
        crate::command_checks::check_finite(torques)?;
        let (motion_generator_mode, _) = self.network.latest_state_modes();
        if motion_generator_mode != MotionGeneratorMode::Idle as u8 && motion_generator_mode != MotionGeneratorMode::None as u8 {
            return Err(FrankaError::InvalidOperation {
                message: "a motion generator is still running; cannot finish external torque control".into(),
            });
        }

        let robot_cmd = RobotCommand {
            message_id: self.network.latest_state_message_id(),
            motion: crate::motion_conversion::empty_command(),
            control: ControllerCommand {
                tau_j_d: **torques,
                torque_command_finished: 1,
            },
        };
        control_loop::finish_motion(self.network, self.motion_id, robot_cmd)?;
        self.finished = true;
        Ok(())
    }
}

impl Drop for ActiveTorqueControl<'_> {
    /// Cancels the motion if it was not finished.
    fn drop(&mut self) {
        if !self.finished {
            self.finished = true;
            let _ = control_loop::cancel_motion(self.network, self.motion_id);
        }
    }
}

/// Active motion control session — read state and write motion commands without a callback.
///
/// Created via `Robot::start_motion_control::<M>()`. The motion is started on creation. End it with
/// [`ActiveMotionControl::finish`]; dropping it unfinished cancels the motion (StopMove), as
/// libfranka does.
pub struct ActiveMotionControl<'a, M: MotionType> {
    network: &'a mut Network,
    motion_id: u32,
    /// Controller the motion was started with; decides whether torques must accompany commands.
    controller_mode: ControllerMode,
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
            controller_mode,
            finished: false,
            _marker: std::marker::PhantomData,
        })
    }

    /// Read the latest robot state.
    ///
    /// Returns an error once the robot has left the motion, as libfranka's `readOnce` does, and,
    /// without reading, once the session has finished.
    pub fn read_state(&mut self) -> FrankaResult<RobotState> {
        if self.finished {
            return Err(finished_error());
        }
        let state = control_loop::receive_robot_state(self.network)?;
        control_loop::check_motion_error(&state, self.motion_id, self.network)?;
        Ok(state)
    }

    /// Returns an error unless this session accepts a command: not finished, and torques given
    /// exactly when the motion was started with the external controller (libfranka's `writeOnce`
    /// checks).
    fn check_writable(&self, with_torques: bool) -> FrankaResult<()> {
        if self.finished {
            return Err(finished_error());
        }
        let external = self.controller_mode == ControllerMode::ExternalController;
        if with_torques && !external {
            return Err(FrankaError::InvalidOperation {
                message: "torques can only be commanded with the ExternalController mode".into(),
            });
        }
        if !with_torques && external {
            return Err(FrankaError::InvalidOperation {
                message: "torque command missing: the motion was started with ExternalController".into(),
            });
        }
        Ok(())
    }

    /// Returns the robot command for `motion` and optional `torques`, after libfranka's checks
    /// (finite values, a homogeneous pose, a valid elbow).
    fn robot_command(&self, motion: &M, torques: Option<&Torques>) -> FrankaResult<RobotCommand> {
        let motion = motion.active_command()?;
        let tau_j_d = match torques {
            Some(torques) => {
                crate::command_checks::check_finite(torques)?;
                **torques
            }
            None => [0.0; 7],
        };
        Ok(RobotCommand {
            message_id: self.network.latest_state_message_id(),
            motion,
            control: ControllerCommand { tau_j_d, torque_command_finished: 0 },
        })
    }

    /// Sends `motion` to the robot and returns the robot state received after sending.
    ///
    /// Returns an error, without sending, if the session has finished, was started with the
    /// external controller (which needs [`ActiveMotionControl::write_motion_with_torques`]), or the
    /// command is invalid (non-finite value, non-homogeneous pose, elbow sign not ±1).
    pub fn write_motion(&mut self, motion: &M) -> FrankaResult<RobotState> {
        self.check_writable(false)?;
        let robot_cmd = self.robot_command(motion, None)?;
        control_loop::send_robot_command(self.network, &robot_cmd)?;
        self.read_state()
    }

    /// Sends `motion` and `torques` together and returns the robot state received after sending.
    ///
    /// Returns an error, without sending, if the session has finished, was not started with the
    /// external controller, or a command is invalid.
    pub fn write_motion_with_torques(&mut self, motion: &M, torques: &Torques) -> FrankaResult<RobotState> {
        self.check_writable(true)?;
        let robot_cmd = self.robot_command(motion, Some(torques))?;
        control_loop::send_robot_command(self.network, &robot_cmd)?;
        self.read_state()
    }

    /// Ends the motion with `motion` as the final command, as libfranka's `writeOnce` with
    /// `motion_finished` set does: the command is sent with its finished flag while the robot still
    /// reports the motion running, then the Move response is awaited.
    ///
    /// Returns an error, without sending, under the same conditions as
    /// [`ActiveMotionControl::write_motion`]. If finishing fails, the motion stays unfinished and
    /// dropping it cancels the motion.
    pub fn finish(&mut self, motion: &M) -> FrankaResult<()> {
        self.check_writable(false)?;
        let robot_cmd = self.robot_command(motion, None)?;
        self.finish_with(robot_cmd)
    }

    /// Ends a motion started with the external controller, with `motion` and `torques` as the
    /// final commands. As in libfranka, only the motion command carries the finished flag.
    ///
    /// Returns an error, without sending, under the same conditions as
    /// [`ActiveMotionControl::write_motion_with_torques`].
    pub fn finish_with_torques(&mut self, motion: &M, torques: &Torques) -> FrankaResult<()> {
        self.check_writable(true)?;
        let robot_cmd = self.robot_command(motion, Some(torques))?;
        self.finish_with(robot_cmd)
    }

    /// Sets the motion-finished flag of `robot_cmd`, finishes the motion with it, and marks the
    /// session finished on success.
    fn finish_with(&mut self, mut robot_cmd: RobotCommand) -> FrankaResult<()> {
        robot_cmd.motion.motion_generation_finished = 1;
        control_loop::finish_motion(self.network, self.motion_id, robot_cmd)?;
        self.finished = true;
        Ok(())
    }
}

impl<M: MotionType> Drop for ActiveMotionControl<'_, M> {
    /// Cancels the motion if it was not finished.
    fn drop(&mut self) {
        if !self.finished {
            self.finished = true;
            let _ = control_loop::cancel_motion(self.network, self.motion_id);
        }
    }
}

/// Error for a read, write or finish after the session has finished (libfranka: "writeOnce must not be
/// called after the motion has finished").
fn finished_error() -> FrankaError {
    FrankaError::InvalidOperation {
        message: "the motion has already finished".into(),
    }
}
