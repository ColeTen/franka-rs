//! Hardware check of the model's forward kinematics against the pose the robot reports.

use franka_rs::model::RobotModel;
use franka_rs::robot::Robot;
use franka_rs::types::{CartesianPose, Frame};

/// Largest accepted distance between the modeled and reported end-effector positions, in m.
const POSITION_TOLERANCE: f64 = 1e-3;

#[test]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut robot = Robot::connect("192.168.1.10")?;
    let model = robot.load_model()?;

    let state = robot.read_once()?;
    let modeled_pose = model.pose_from_state(Frame::EndEffector, &state);
    let reported_pose = CartesianPose::from_column_major(&state.o_t_ee).inner;
    let position_error =
        (modeled_pose.translation.vector - reported_pose.translation.vector).norm();
    println!("modeled: {modeled_pose}\nreported: {reported_pose}\nposition error: {position_error} m");
    assert!(position_error < POSITION_TOLERANCE);

    let j_body = model.body_jacobian_from_state(Frame::EndEffector, &state);
    println!("{j_body}");
    Ok(())
}
