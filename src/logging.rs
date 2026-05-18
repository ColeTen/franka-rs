use crate::robot_state::RobotState;
use crate::wire::robot::RobotCommand;

/// A single entry in the control loop log.
#[derive(Debug, Clone)]
pub struct LogEntry {
    pub state: RobotState,
    pub command: Option<RobotCommand>,
}

/// Fixed-size ring buffer for logging robot state and commands during control.
///
/// When the buffer is full, the oldest entries are overwritten.
/// Used primarily for post-mortem diagnostics when a `ControlException` occurs.
pub struct Logger {
    buffer: Vec<Option<LogEntry>>,
    write_index: usize,
    count: usize,
}

impl Logger {
    /// Default ring buffer capacity (stores ~1 second at 1kHz).
    pub const DEFAULT_CAPACITY: usize = 1000;

    pub fn new(capacity: usize) -> Self {
        let mut buffer = Vec::with_capacity(capacity);
        buffer.resize_with(capacity, || None);
        Self {
            buffer,
            write_index: 0,
            count: 0,
        }
    }

    pub fn log(&mut self, state: RobotState, command: Option<RobotCommand>) {
        self.buffer[self.write_index] = Some(LogEntry { state, command });
        self.write_index = (self.write_index + 1) % self.buffer.len();
        if self.count < self.buffer.len() {
            self.count += 1;
        }
    }

    /// Returns log entries in chronological order (oldest first).
    pub fn flush(&self) -> Vec<LogEntry> {
        let cap = self.buffer.len();
        let mut result = Vec::with_capacity(self.count);

        if self.count < cap {
            for e in self.buffer[..self.count].iter().flatten() {
                result.push(e.clone());
            }
        } else {
            for i in 0..cap {
                let idx = (self.write_index + i) % cap;
                if let Some(e) = &self.buffer[idx] {
                    result.push(e.clone());
                }
            }
        }

        result
    }

    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn clear(&mut self) {
        for entry in &mut self.buffer {
            *entry = None;
        }
        self.write_index = 0;
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::errors::RobotErrors;
    use crate::types::{MotionGeneratorMode, RobotMode};

    fn dummy_state() -> RobotState {
        RobotState {
            o_t_ee: [0.0; 16],
            o_t_ee_d: [0.0; 16],
            f_t_ee: [0.0; 16],
            ee_t_k: [0.0; 16],
            f_t_ne: [0.0; 16],
            ne_t_ee: [0.0; 16],
            m_ee: 0.0,
            i_ee: [0.0; 9],
            f_x_cee: [0.0; 3],
            m_load: 0.0,
            i_load: [0.0; 9],
            f_x_cload: [0.0; 3],
            elbow: [0.0; 2],
            elbow_d: [0.0; 2],
            elbow_c: [0.0; 2],
            delbow_c: [0.0; 2],
            ddelbow_c: [0.0; 2],
            tau_j: [0.0; 7],
            tau_j_d: [0.0; 7],
            dtau_j: [0.0; 7],
            q: [0.0; 7],
            q_d: [0.0; 7],
            dq: [0.0; 7],
            dq_d: [0.0; 7],
            ddq_d: [0.0; 7],
            joint_contact: [0.0; 7],
            cartesian_contact: [0.0; 6],
            joint_collision: [0.0; 7],
            cartesian_collision: [0.0; 6],
            tau_ext_hat_filtered: [0.0; 7],
            o_f_ext_hat_k: [0.0; 6],
            k_f_ext_hat_k: [0.0; 6],
            o_dp_ee_d: [0.0; 6],
            o_ddp_o: [0.0; 3],
            o_t_ee_c: [0.0; 16],
            o_dp_ee_c: [0.0; 6],
            o_ddp_ee_c: [0.0; 6],
            theta: [0.0; 7],
            dtheta: [0.0; 7],
            current_errors: RobotErrors::empty(),
            last_motion_errors: RobotErrors::empty(),
            control_command_success_rate: 1.0,
            robot_mode: RobotMode::Idle,
            motion_generator_mode: MotionGeneratorMode::Idle,
            time: std::time::Duration::ZERO,
        }
    }

    #[test]
    fn empty_logger() {
        let logger = Logger::new(10);
        assert!(logger.is_empty());
        assert_eq!(logger.len(), 0);
        assert!(logger.flush().is_empty());
    }

    #[test]
    fn log_and_flush() {
        let mut logger = Logger::new(5);
        for _ in 0..3 {
            logger.log(dummy_state(), None);
        }
        assert_eq!(logger.len(), 3);
        let entries = logger.flush();
        assert_eq!(entries.len(), 3);
    }

    #[test]
    fn ring_buffer_wraps() {
        let mut logger = Logger::new(3);
        for _ in 0..5 {
            logger.log(dummy_state(), None);
        }
        assert_eq!(logger.len(), 3);
        let entries = logger.flush();
        assert_eq!(entries.len(), 3);
    }

    #[test]
    fn clear_resets() {
        let mut logger = Logger::new(10);
        logger.log(dummy_state(), None);
        logger.clear();
        assert!(logger.is_empty());
        assert!(logger.flush().is_empty());
    }
}
