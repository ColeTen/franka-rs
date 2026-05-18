/// Sample time of the robot control loop (1 kHz).
pub const DELTA_T: f64 = 1e-3;

/// Number of joints on the Franka robot.
pub const NUM_JOINTS: usize = 7;

/// Robot command port (TCP).
pub const ROBOT_COMMAND_PORT: u16 = 1337;

/// Gripper command port (TCP).
pub const GRIPPER_COMMAND_PORT: u16 = 1338;

/// Vacuum gripper command port (TCP).
pub const VACUUM_GRIPPER_COMMAND_PORT: u16 = 1339;

/// Robot protocol version.
pub const ROBOT_PROTOCOL_VERSION: u16 = 10;

/// Gripper protocol version.
pub const GRIPPER_PROTOCOL_VERSION: u16 = 3;

/// Vacuum gripper protocol version.
pub const VACUUM_GRIPPER_PROTOCOL_VERSION: u16 = 1;

/// Default network timeout in milliseconds.
pub const DEFAULT_TIMEOUT_MS: u64 = 1000;

/// Default TCP keepalive idle time in seconds.
pub const KEEPALIVE_IDLE_SECS: u64 = 1;

/// Default TCP keepalive interval in seconds.
pub const KEEPALIVE_INTERVAL_SECS: u64 = 3;

/// Default TCP keepalive probe count.
pub const KEEPALIVE_PROBE_COUNT: u32 = 1;
