//! Continuously reads and prints the robot state from a connected Franka Research 3, without
//! commanding any motion.
//!
//! Mirrors libfranka's `echo_robot_state` example (connect, then read and print 101 states), so the
//! network traffic of the two can be captured and compared:
//!
//! ```text
//! cargo run --example echo_robot_state <robot-ip>
//! ```

use franka_rs::robot::Robot;

/// Number of states after which reading stops; libfranka's example reads one more than this.
const STATE_COUNT: usize = 100;

/// Connects to the robot whose IP is given as the first argument and prints its state stream.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot_ip = std::env::args()
        .nth(1)
        .ok_or("usage: echo_robot_state <robot-ip>")?;

    let robot = Robot::connect(&robot_ip)?;

    let mut count = 0;
    robot.read(|robot_state| {
        println!("{robot_state:?}");
        let keep_reading = count < STATE_COUNT;
        count += 1;
        keep_reading
    })?;

    println!("Done.");
    Ok(())
}