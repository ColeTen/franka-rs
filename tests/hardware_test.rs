//! Hardware check of the model's forward kinematics against the pose the robot reports.
//!
//! The robot is identified by the FCI connection handshake, not by its address: if nothing at the
//! address completes the handshake, the test skips rather than failing or running against the wrong
//! device. A robot that answers but is version-incompatible fails the test.

use franka_rs::errors::FrankaError;
use franka_rs::model::RobotModel;
use franka_rs::robot::Robot;
use franka_rs::types::{CartesianPose, Frame};

/// Largest accepted distance between the modeled and reported end-effector positions, in m.
const POSITION_TOLERANCE: f64 = 1e-3;

#[test]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut robot = match Robot::connect("192.168.1.10") {
        Ok(robot) => robot,
        Err(FrankaError::Network { .. }) => {
            eprintln!("skipping: no robot reachable");
            return Ok(());
        }
        Err(FrankaError::Protocol { .. }) => {
            eprintln!("skipping: device did not complete the FCI handshake; not a Franka robot");
            return Ok(());
        }
        // A robot answered but franka-rs cannot talk to it (e.g. incompatible version): a real defect.
        Err(error) => return Err(error.into()),
    };
    let model = robot.load_model()?;

    let state = robot.read_once()?;
    let modeled_pose = model.pose_from_state(Frame::EndEffector, &state);
    let reported_pose = CartesianPose::from_column_major(&state.o_t_ee).to_isometry()?;
    let position_error =
        (modeled_pose.translation.vector - reported_pose.translation.vector).norm();
    println!("modeled: {modeled_pose}\nreported: {reported_pose}\nposition error: {position_error} m");
    assert!(position_error < POSITION_TOLERANCE);

    let j_body = model.body_jacobian_from_state(Frame::EndEffector, &state);
    println!("{j_body}");
    Ok(())
}
