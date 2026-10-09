//! Measures network performance with a connected Franka Research 3: moves the robot to a start
//! configuration, then runs ten seconds of zero-torque control and reports the control command
//! success rate.
//!
//! Mirrors libfranka's `communication_test` example, so the network traffic of the two can be
//! captured and compared:
//!
//! ```text
//! cargo run --example communication_test <robot-ip>
//! ```
//!
//! WARNING: this program moves the robot. Make sure there is enough space in front of it and keep
//! the user stop button at hand.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};
use std::ops::ControlFlow;
use std::time::Duration;

use franka_rs::control_types::MotionResult;
use franka_rs::robot::Robot;
use franka_rs::robot::config::{CollisionConfig, MotionConfig};
use franka_rs::robot_state::RobotState;
use franka_rs::types::{JointPositions, Torques};

/// Joint configuration the robot moves to before the communication test.
const START_CONFIGURATION: [f64; 7] = [0.0, -FRAC_PI_4, 0.0, -3.0 * FRAC_PI_4, 0.0, FRAC_PI_2, FRAC_PI_4];
/// Fraction of the motion generator's maximum velocity and acceleration used for the move.
const SPEED_FACTOR: f64 = 0.5;
/// Duration of the zero-torque phase in milliseconds.
const TEST_DURATION_MS: u128 = 10_000;

/// Connects to the robot whose IP is given as the first argument and runs the test.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot_ip = std::env::args()
        .nth(1)
        .ok_or("usage: communication_test <robot-ip>")?;

    let mut robot = Robot::connect(&robot_ip)?;
    set_default_behavior(&mut robot)?;

    let mut motion_generator = MotionGenerator::new(SPEED_FACTOR, START_CONFIGURATION);
    println!("WARNING: This example will move the robot! Please make sure to have the user stop button at hand!");
    println!("Press Enter to continue...");
    std::io::stdin().read_line(&mut String::new())?;
    // libfranka's Robot::control defaults to no rate limiting; match it.
    let motion_config = MotionConfig::default().with_rate_limiting(false);
    robot.control_joint_positions(&motion_config, |state, period| motion_generator.next(state, period))?;
    println!("Finished moving to initial joint configuration.\n");
    println!("Starting communication test.");

    let torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0];
    let force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0];
    robot.set_collision_behavior(&CollisionConfig::symmetric(
        torque_thresholds,
        torque_thresholds,
        force_thresholds,
        force_thresholds,
    ))?;

    let zero_torques = Torques::new([0.0; 7]);
    let mut counter: u64 = 0;
    let mut time_ms: u128 = 0;
    let mut average_success_rate = 0.0;
    let mut minimum_success_rate: f64 = 1.0;
    let mut maximum_success_rate: f64 = 0.0;

    let mut control = robot.start_torque_control()?;
    let mut state = control.read_state()?;
    let mut previous_time = state.time;
    loop {
        time_ms += state.time.saturating_sub(previous_time).as_millis();
        previous_time = state.time;
        if time_ms == 0 {
            state = control.write_torques(&zero_torques)?;
            continue;
        }
        counter += 1;

        let success_rate = state.control_command_success_rate;
        if counter % 100 == 0 {
            println!("#{counter} Current success rate: {success_rate:.2}");
        }
        std::thread::sleep(Duration::from_micros(100));

        average_success_rate += success_rate;
        maximum_success_rate = maximum_success_rate.max(success_rate);
        minimum_success_rate = minimum_success_rate.min(success_rate);

        if time_ms >= TEST_DURATION_MS {
            println!("\nFinished test, shutting down example");
            control.finish(&zero_torques)?;
            break;
        }
        // Sending zero torques - if the end effector is configured correctly, the robot should not move.
        state = control.write_torques(&zero_torques)?;
    }

    average_success_rate /= counter as f64;

    println!("\n\n#######################################################");
    let lost_robot_states = time_ms.saturating_sub(u128::from(counter));
    if lost_robot_states > 0 {
        println!("The control loop did not get executed {lost_robot_states} times in the");
        println!("last {time_ms} milliseconds! (lost {lost_robot_states} robot states)\n");
    }
    println!("Control command success rate of {counter} samples: ");
    println!("Max: {maximum_success_rate:.2}");
    println!("Avg: {average_success_rate:.2}");
    println!("Min: {minimum_success_rate:.2}");
    if average_success_rate < 0.90 {
        println!("\nWARNING: THIS SETUP IS PROBABLY NOT SUFFICIENT FOR FCI!");
        println!("PLEASE TRY OUT A DIFFERENT PC / NIC");
    } else if average_success_rate < 0.95 {
        println!("\nWARNING: MANY PACKETS GOT LOST!");
        println!("PLEASE INSPECT YOUR SETUP AND FOLLOW ADVICE ON");
        println!("https://frankarobotics.github.io/docs/troubleshooting.html");
    }
    println!("#######################################################\n");

    Ok(())
}

/// Sets the collision thresholds and impedances that libfranka's examples use by default.
fn set_default_behavior(robot: &mut Robot) -> Result<(), Box<dyn std::error::Error>> {
    robot.set_collision_behavior(&CollisionConfig {
        lower_torque_thresholds_acceleration: [20.0; 7],
        upper_torque_thresholds_acceleration: [20.0; 7],
        lower_torque_thresholds_nominal: [10.0; 7],
        upper_torque_thresholds_nominal: [10.0; 7],
        lower_force_thresholds_acceleration: [20.0; 6],
        upper_force_thresholds_acceleration: [20.0; 6],
        lower_force_thresholds_nominal: [10.0; 6],
        upper_force_thresholds_nominal: [10.0; 6],
    })?;
    robot.set_joint_impedance([3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0])?;
    robot.set_cartesian_impedance([3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0])?;
    Ok(())
}

/// Joint-space motion to a goal configuration whose velocity profile satisfies per-joint velocity
/// and acceleration limits, with all joints arriving at the same time. A port of the
/// `MotionGenerator` in libfranka's `examples_common`.
struct MotionGenerator {
    /// Goal joint configuration in rad.
    q_goal: [f64; 7],
    /// Maximum joint velocities in rad/s.
    dq_max: [f64; 7],
    /// Maximum joint accelerations while speeding up, in rad/s².
    ddq_max_start: [f64; 7],
    /// Maximum joint accelerations while slowing down, in rad/s².
    ddq_max_goal: [f64; 7],
    /// Seconds since the motion started.
    time: f64,
    /// Joint configuration at the start of the motion.
    q_start: [f64; 7],
    /// Total joint displacement from start to goal.
    delta_q: [f64; 7],
    /// Per-joint cruise velocity, scaled so all joints finish together.
    dq_max_sync: [f64; 7],
    /// Per-joint time at which the speed-up phase ends.
    t_1_sync: [f64; 7],
    /// Per-joint time at which the slow-down phase begins.
    t_2_sync: [f64; 7],
    /// Per-joint time at which the motion ends.
    t_f_sync: [f64; 7],
    /// Per-joint displacement covered by the end of the speed-up phase.
    q_1: [f64; 7],
}

impl MotionGenerator {
    /// Displacement below which a joint is considered to have no motion left.
    const DELTA_Q_MOTION_FINISHED: f64 = 1e-6;

    /// Creates a generator moving to `q_goal`, with velocity and acceleration limits scaled by
    /// `speed_factor`.
    fn new(speed_factor: f64, q_goal: [f64; 7]) -> Self {
        Self {
            q_goal,
            dq_max: [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5].map(|value| value * speed_factor),
            ddq_max_start: [5.0; 7].map(|value| value * speed_factor),
            ddq_max_goal: [5.0; 7].map(|value| value * speed_factor),
            time: 0.0,
            q_start: [0.0; 7],
            delta_q: [0.0; 7],
            dq_max_sync: [0.0; 7],
            t_1_sync: [0.0; 7],
            t_2_sync: [0.0; 7],
            t_f_sync: [0.0; 7],
            q_1: [0.0; 7],
        }
    }

    /// Advances the motion by `period` and returns the commanded joint positions, as `Break` once
    /// every joint has reached the goal.
    fn next(&mut self, state: &RobotState, period: Duration) -> MotionResult<JointPositions> {
        self.time += period.as_secs_f64();
        if self.time == 0.0 {
            self.q_start = state.q;
            self.delta_q = std::array::from_fn(|joint| self.q_goal[joint] - self.q_start[joint]);
            self.calculate_synchronized_values();
        }

        let (delta_q_d, motion_finished) = self.calculate_desired_values(self.time);
        let positions = JointPositions::new(std::array::from_fn(|joint| self.q_start[joint] + delta_q_d[joint]));
        if motion_finished {
            ControlFlow::Break(positions)
        } else {
            ControlFlow::Continue(positions)
        }
    }

    /// Returns the displacement from the start configuration at `time`, and whether every joint
    /// has finished.
    fn calculate_desired_values(&self, time: f64) -> ([f64; 7], bool) {
        let mut delta_q_d = [0.0; 7];
        let mut all_finished = true;
        for joint in 0..7 {
            let sign = sign(self.delta_q[joint]);
            let t_1 = self.t_1_sync[joint];
            let t_2 = self.t_2_sync[joint];
            let t_f = self.t_f_sync[joint];
            let t_d = t_2 - t_1;
            let delta_t_2 = t_f - t_2;
            let dq_max_sync = self.dq_max_sync[joint];

            if self.delta_q[joint].abs() < Self::DELTA_Q_MOTION_FINISHED {
                delta_q_d[joint] = 0.0;
            } else if time < t_1 {
                delta_q_d[joint] = -1.0 / t_1.powi(3) * dq_max_sync * sign * (0.5 * time - t_1) * time.powi(3);
                all_finished = false;
            } else if time < t_2 {
                delta_q_d[joint] = self.q_1[joint] + (time - t_1) * dq_max_sync * sign;
                all_finished = false;
            } else if time < t_f {
                delta_q_d[joint] = self.delta_q[joint]
                    + 0.5
                        * (1.0 / delta_t_2.powi(3)
                            * (time - t_1 - 2.0 * delta_t_2 - t_d)
                            * (time - t_1 - t_d).powi(3)
                            + (2.0 * time - 2.0 * t_1 - delta_t_2 - 2.0 * t_d))
                        * dq_max_sync
                        * sign;
                all_finished = false;
            } else {
                delta_q_d[joint] = self.delta_q[joint];
            }
        }
        (delta_q_d, all_finished)
    }

    /// Computes each joint's cruise velocity and phase times so that all joints finish together.
    fn calculate_synchronized_values(&mut self) {
        let mut dq_max_reach = self.dq_max;
        let mut t_f = [0.0; 7];
        for joint in 0..7 {
            let delta_q = self.delta_q[joint];
            if delta_q.abs() > Self::DELTA_Q_MOTION_FINISHED {
                let (dq_max, start, goal) = (self.dq_max[joint], self.ddq_max_start[joint], self.ddq_max_goal[joint]);
                if delta_q.abs() < 3.0 / 4.0 * (dq_max.powi(2) / start) + 3.0 / 4.0 * (dq_max.powi(2) / goal) {
                    dq_max_reach[joint] =
                        (4.0 / 3.0 * delta_q * sign(delta_q) * (start * goal) / (start + goal)).sqrt();
                }
                let t_1 = 1.5 * dq_max_reach[joint] / start;
                let delta_t_2 = 1.5 * dq_max_reach[joint] / goal;
                t_f[joint] = t_1 / 2.0 + delta_t_2 / 2.0 + delta_q.abs() / dq_max_reach[joint];
            }
        }

        let max_t_f = t_f.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        for joint in 0..7 {
            let delta_q = self.delta_q[joint];
            if delta_q.abs() > Self::DELTA_Q_MOTION_FINISHED {
                let (start, goal) = (self.ddq_max_start[joint], self.ddq_max_goal[joint]);
                let a = 1.5 / 2.0 * (goal + start);
                let b = -1.0 * max_t_f * goal * start;
                let c = delta_q.abs() * goal * start;
                let delta = (b * b - 4.0 * a * c).max(0.0);
                self.dq_max_sync[joint] = (-1.0 * b - delta.sqrt()) / (2.0 * a);
                self.t_1_sync[joint] = 1.5 * self.dq_max_sync[joint] / start;
                let delta_t_2_sync = 1.5 * self.dq_max_sync[joint] / goal;
                self.t_f_sync[joint] =
                    self.t_1_sync[joint] / 2.0 + delta_t_2_sync / 2.0 + (delta_q / self.dq_max_sync[joint]).abs();
                self.t_2_sync[joint] = self.t_f_sync[joint] - delta_t_2_sync;
                self.q_1[joint] = self.dq_max_sync[joint] * sign(delta_q) * (0.5 * self.t_1_sync[joint]);
            }
        }
    }
}

/// Returns −1, 0, or 1 according to the sign of `value`, matching Eigen's `cwiseSign`.
fn sign(value: f64) -> f64 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}