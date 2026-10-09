//! Downloads the robot model URDF from a connected Franka Research 3 and prints it to stdout.
//!
//! The printed URDF is the exact document libfranka uses at runtime and is intended to be saved
//! as a tracked fixture for the model validation tests, for example:
//!
//! ```text
//! cargo run --example download_urdf 192.168.1.10 > tests/fixtures/fr3_robot.urdf
//! ```

use franka_rs::robot::Robot;

/// Connects to the robot whose IP is given as the first argument and prints its URDF.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot_ip = std::env::args().nth(1).ok_or(
        "usage: download_urdf <robot-ip>\n\
         prints the robot model URDF to stdout; redirect it to a file to save it",
    )?;

    let mut robot = Robot::connect(&robot_ip)?;
    let urdf = robot.get_robot_model()?;
    print!("{urdf}");

    Ok(())
}
