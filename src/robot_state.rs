use crate::errors::RobotErrors;
use crate::types::{MotionGeneratorMode, RobotMode};

/// Complete robot state received from the robot at each control cycle.
///
/// All pose transformations are 4x4 column-major homogeneous matrices.
/// Joint arrays have 7 elements (one per joint).
#[derive(Debug, Clone)]
pub struct RobotState {
    /// Measured end effector pose in base frame (O_T_EE).
    pub o_t_ee: [f64; 16],

    /// Last desired end effector pose from motion generation (O_T_EE_d).
    pub o_t_ee_d: [f64; 16],

    /// End effector frame pose in flange frame (F_T_EE).
    pub f_t_ee: [f64; 16],

    /// Stiffness frame pose in end effector frame (EE_T_K).
    pub ee_t_k: [f64; 16],

    /// Nominal end effector frame pose in flange frame (F_T_NE).
    pub f_t_ne: [f64; 16],

    /// End effector frame pose in nominal end effector frame (NE_T_EE).
    pub ne_t_ee: [f64; 16],

    /// Configured end effector mass in kg.
    pub m_ee: f64,

    /// Configured end effector rotational inertia matrix (3x3, column-major).
    pub i_ee: [f64; 9],

    /// Configured center of mass of end effector in flange frame.
    pub f_x_cee: [f64; 3],

    /// Configured external load mass in kg.
    pub m_load: f64,

    /// Configured external load rotational inertia matrix (3x3, column-major).
    pub i_load: [f64; 9],

    /// Configured center of mass of external load in flange frame.
    pub f_x_cload: [f64; 3],

    /// Elbow configuration: [joint3_position, flip_direction].
    pub elbow: [f64; 2],

    /// Desired elbow configuration.
    pub elbow_d: [f64; 2],

    /// Commanded elbow configuration.
    pub elbow_c: [f64; 2],

    /// Commanded elbow velocity.
    pub delbow_c: [f64; 2],

    /// Commanded elbow acceleration.
    pub ddelbow_c: [f64; 2],

    /// Measured link-side joint torques in Nm.
    pub tau_j: [f64; 7],

    /// Desired link-side joint torques (without gravity) in Nm.
    pub tau_j_d: [f64; 7],

    /// Derivative of measured joint torques in Nm/s.
    pub dtau_j: [f64; 7],

    /// Measured joint positions in rad.
    pub q: [f64; 7],

    /// Desired joint positions in rad.
    pub q_d: [f64; 7],

    /// Measured joint velocities in rad/s.
    pub dq: [f64; 7],

    /// Desired joint velocities in rad/s.
    pub dq_d: [f64; 7],

    /// Desired joint accelerations in rad/s^2.
    pub ddq_d: [f64; 7],

    /// Joint contact levels (0 = no contact).
    pub joint_contact: [f64; 7],

    /// Cartesian contact levels (x, y, z, R, P, Y).
    pub cartesian_contact: [f64; 6],

    /// Joint collision levels (persists until reset).
    pub joint_collision: [f64; 7],

    /// Cartesian collision levels (persists until reset).
    pub cartesian_collision: [f64; 6],

    /// Filtered external torques on joints in Nm.
    pub tau_ext_hat_filtered: [f64; 7],

    /// Estimated external wrench in base frame (O_F_ext_hat_K) in [N, N, N, Nm, Nm, Nm].
    pub o_f_ext_hat_k: [f64; 6],

    /// Estimated external wrench in stiffness frame (K_F_ext_hat_K) in [N, N, N, Nm, Nm, Nm].
    pub k_f_ext_hat_k: [f64; 6],

    /// Desired end effector twist in base frame (O_dP_EE_d).
    pub o_dp_ee_d: [f64; 6],

    /// Base acceleration (linear component) in base frame (O_ddP_O).
    pub o_ddp_o: [f64; 3],

    /// Last commanded end effector pose (O_T_EE_c).
    pub o_t_ee_c: [f64; 16],

    /// Last commanded end effector twist (O_dP_EE_c).
    pub o_dp_ee_c: [f64; 6],

    /// Last commanded end effector acceleration (O_ddP_EE_c).
    pub o_ddp_ee_c: [f64; 6],

    /// Motor positions in rad.
    pub theta: [f64; 7],

    /// Motor velocities in rad/s.
    pub dtheta: [f64; 7],

    /// Current error state.
    pub current_errors: RobotErrors,

    /// Errors that aborted the previous motion.
    pub last_motion_errors: RobotErrors,

    /// Percentage of last 100 control commands successfully received. Range: [0, 1].
    pub control_command_success_rate: f64,

    /// Current robot mode.
    pub robot_mode: RobotMode,

    /// Current motion generator mode.
    pub motion_generator_mode: MotionGeneratorMode,

    /// Strictly monotonically increasing timestamp since robot start.
    pub time: std::time::Duration,
}

