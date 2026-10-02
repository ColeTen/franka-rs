//! Tests of [`Model`] against reference values computed for the FR3 URDF.

use std::f64::consts::FRAC_1_SQRT_2;

use nalgebra::{Isometry3, Matrix3, Matrix4, Translation3, UnitQuaternion, Vector3};

use super::*;

/// Tolerance for exact reference values.
const EXACT_TOLERANCE: f64 = 1e-5;
/// Tolerance for forward-kinematics reference values printed to six decimals.
const POSE_TOLERANCE: f64 = 1e-3;
/// Tolerance for approximate baseline gravity values, which are not exact for this URDF.
const GRAVITY_TOLERANCE: f64 = 0.2;
/// Tolerance for approximate baseline Coriolis values, which are not exact for this URDF.
const CORIOLIS_TOLERANCE: f64 = 0.2;
/// Tolerance for approximate baseline mass-matrix values, which are not exact for this URDF.
const MASS_TOLERANCE: f64 = 0.1;

/// Earth gravity in the base frame.
const EARTH_GRAVITY: [f64; 3] = [0.0, 0.0, -9.81];

/// Returns the model of the FR3 test URDF.
fn fr3_model() -> Model {
    Model::from_urdf(include_str!("../../external/libfranka/test/fr3.urdf")).unwrap()
}

/// Returns the default "ready" joint configuration.
fn default_configuration() -> JointVector {
    JointVector::from([0.0, 0.0, 0.0, -0.75 * std::f64::consts::PI, 0.0, 0.75 * std::f64::consts::PI, 0.0])
}

/// Returns a configuration slightly offset from the default one.
fn moved_configuration() -> JointVector {
    JointVector::from([0.0010, 0.0010, 0.0010, -2.3552, 0.0010, 2.3572, 0.0010])
}

/// Returns the inertia of the Franka Hand end effector.
fn franka_hand() -> RigidBodyInertia {
    RigidBodyInertia::new(
        0.73,
        Vector3::new(0.01, 0.0, 0.03),
        Matrix3::from_diagonal(&Vector3::new(0.001, 0.0025, 0.0017)),
    )
}

/// Asserts that `actual` and `expected` (both column-major) agree element-wise within `tolerance`.
fn assert_close(actual: &[f64], expected: &[f64], tolerance: f64) {
    assert_eq!(actual.len(), expected.len());
    for (index, (actual_value, expected_value)) in actual.iter().zip(expected).enumerate() {
        assert!(
            (actual_value - expected_value).abs() <= tolerance,
            "index {index}: actual {actual_value}, expected {expected_value}"
        );
    }
}

/// Returns the pose of `frame` with identity end-effector and stiffness offsets.
fn pose(model: &Model, frame: Frame, q: &JointVector) -> Matrix4<f64> {
    model
        .pose(frame, q, &Isometry3::identity(), &Isometry3::identity())
        .to_homogeneous()
}

#[test]
fn joint_and_flange_poses_at_zero_match_reference() {
    let model = fr3_model();
    let q = JointVector::zeros();
    let expected: [(Frame, [f64; 16]); 8] = [
        (Frame::Joint1, [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0.333, 1.]),
        (Frame::Joint2, [1., 0., 0., 0., 0., 0., -1., 0., 0., 1., 0., 0., 0., 0., 0.333, 1.]),
        (Frame::Joint3, [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0.6492, 1.]),
        (Frame::Joint4, [1., 0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., 0.0825, 0., 0.6492, 1.]),
        (Frame::Joint5, [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 1.0334, 1.]),
        (Frame::Joint6, [1., 0., 0., 0., 0., 0., 1., 0., 0., -1., 0., 0., 0., 0., 1.0334, 1.]),
        (Frame::Joint7, [1., 0., 0., 0., 0., -1., 0., 0., 0., 0., -1., 0., 0.088, 0., 1.0334, 1.]),
        (Frame::Flange, [1., 0., 0., 0., 0., -1., 0., 0., 0., 0., -1., 0., 0.088, 0., 0.9264, 1.]),
    ];
    for (frame, expected_pose) in expected {
        assert_close(pose(&model, frame, &q).as_slice(), &expected_pose, POSE_TOLERANCE);
    }
}

#[test]
fn joint_and_flange_poses_at_default_match_reference() {
    let model = fr3_model();
    let q = default_configuration();
    let expected: [(Frame, [f64; 16]); 4] = [
        (Frame::Joint4, [-FRAC_1_SQRT_2, 0., -FRAC_1_SQRT_2, 0., FRAC_1_SQRT_2, 0., -FRAC_1_SQRT_2, 0., 0., -1., 0., 0., 0.0825, 0., 0.6492, 1.]),
        (Frame::Joint5, [-FRAC_1_SQRT_2, 0., -FRAC_1_SQRT_2, 0., 0., 1., 0., 0., FRAC_1_SQRT_2, 0., -FRAC_1_SQRT_2, 0., 0.412507, 0., 0.435866, 1.]),
        (Frame::Joint7, [1., 0., 0., 0., 0., -1., 0., 0., 0., 0., -1., 0., 0.500507, 0., 0.435866, 1.]),
        (Frame::Flange, [1., 0., 0., 0., 0., -1., 0., 0., 0., 0., -1., 0., 0.500507, 0., 0.328866, 1.]),
    ];
    for (frame, expected_pose) in expected {
        assert_close(pose(&model, frame, &q).as_slice(), &expected_pose, POSE_TOLERANCE);
    }
}

#[test]
fn end_effector_pose_at_moved_configuration_matches_reference() {
    let model = fr3_model();
    let expected = [
        0.999999, 0.000292, 0.000999, 0.0, 0.000293, -1.000000, -0.000709, 0.0, 0.000999, 0.000709,
        -0.999999, 0.0, 0.500928, 0.001015, 0.328869, 1.0,
    ];
    let actual = pose(&model, Frame::EndEffector, &moved_configuration());
    assert_close(actual.as_slice(), &expected, POSE_TOLERANCE);
}

#[test]
fn end_effector_and_stiffness_poses_compose_offsets() {
    let model = fr3_model();
    let q = moved_configuration();
    let f_t_ee = Isometry3::from_parts(Translation3::new(0.1, 0.0, 0.0), UnitQuaternion::from_euler_angles(0.0, 1.0, 0.0));
    let ee_t_k = Isometry3::from_parts(Translation3::new(0.0, 0.02, 0.05), UnitQuaternion::from_euler_angles(0.3, 0.0, 0.0));
    let flange = model.pose(Frame::Flange, &q, &f_t_ee, &ee_t_k);
    let end_effector = model.pose(Frame::EndEffector, &q, &f_t_ee, &ee_t_k);
    let stiffness = model.pose(Frame::Stiffness, &q, &f_t_ee, &ee_t_k);
    assert_close(end_effector.to_homogeneous().as_slice(), (flange * f_t_ee).to_homogeneous().as_slice(), 1e-12);
    assert_close(stiffness.to_homogeneous().as_slice(), (flange * f_t_ee * ee_t_k).to_homogeneous().as_slice(), 1e-12);
}

#[test]
fn zero_jacobian_matches_finite_difference_of_pose() {
    let model = fr3_model();
    let q = moved_configuration();
    let f_t_ee = Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.1034), UnitQuaternion::from_euler_angles(0.0, 0.0, -0.785));
    let ee_t_k = Isometry3::from_parts(Translation3::new(0.05, 0.0, 0.0), UnitQuaternion::identity());
    let step = 1e-7;
    for frame in [Frame::Joint3, Frame::Joint7, Frame::Flange, Frame::EndEffector, Frame::Stiffness] {
        let jacobian = model.zero_jacobian(frame, &q, &f_t_ee, &ee_t_k);
        let base_pose = model.pose(frame, &q, &f_t_ee, &ee_t_k);
        for joint_index in 0..NUM_JOINTS {
            let mut perturbed = q;
            perturbed[joint_index] += step;
            let perturbed_pose = model.pose(frame, &perturbed, &f_t_ee, &ee_t_k);
            let linear = (perturbed_pose.translation.vector - base_pose.translation.vector) / step;
            let angular = (perturbed_pose.rotation * base_pose.rotation.inverse()).scaled_axis() / step;
            let column = jacobian.column(joint_index);
            assert!((column.fixed_rows::<3>(0) - linear).norm() < 1e-5, "{frame:?} linear column {joint_index}");
            assert!((column.fixed_rows::<3>(3) - angular).norm() < 1e-5, "{frame:?} angular column {joint_index}");
        }
    }
}

#[test]
fn body_jacobian_is_zero_jacobian_rotated_into_frame() {
    let model = fr3_model();
    let q = moved_configuration();
    let f_t_ee = Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.1034), UnitQuaternion::from_euler_angles(0.0, 0.0, -0.785));
    let ee_t_k = Isometry3::identity();
    for frame in [Frame::Joint2, Frame::Flange, Frame::EndEffector] {
        let rotation = model.pose(frame, &q, &f_t_ee, &ee_t_k).rotation.to_rotation_matrix();
        let zero = model.zero_jacobian(frame, &q, &f_t_ee, &ee_t_k);
        let body = model.body_jacobian(frame, &q, &f_t_ee, &ee_t_k);
        for joint_index in 0..NUM_JOINTS {
            let zero_column = zero.column(joint_index);
            let body_column = body.column(joint_index);
            let expected_linear = rotation.transpose() * zero_column.fixed_rows::<3>(0);
            let expected_angular = rotation.transpose() * zero_column.fixed_rows::<3>(3);
            assert!((body_column.fixed_rows::<3>(0) - expected_linear).norm() < 1e-12);
            assert!((body_column.fixed_rows::<3>(3) - expected_angular).norm() < 1e-12);
        }
    }
}

#[test]
fn gravity_at_zero_matches_reference() {
    let model = fr3_model();
    let gravity = model.gravity(&JointVector::zeros(), &RigidBodyInertia::zero(), &Vector3::from(EARTH_GRAVITY));
    assert_close(gravity.as_slice(), &[0., -3.52387, 0., -3.44254, 0., 1.63362, 0.], EXACT_TOLERANCE);
}

#[test]
fn gravity_at_zero_with_hand_matches_reference() {
    let model = fr3_model();
    let gravity = model.gravity(&JointVector::zeros(), &franka_hand(), &Vector3::from(EARTH_GRAVITY));
    assert_close(gravity.as_slice(), &[0., -4.22568, 0., -3.33154, 0., 2.33543, 0.], EXACT_TOLERANCE);
}

#[test]
fn gravity_at_default_configurations_matches_reference() {
    let model = fr3_model();
    let gravity = Vector3::from(EARTH_GRAVITY);
    let without_load = model.gravity(&default_configuration(), &RigidBodyInertia::zero(), &gravity);
    assert_close(without_load.as_slice(), &[0., -24.5858, 0., 17.6880, 0.5095, 1.6428, 0.], GRAVITY_TOLERANCE);
    let with_hand = model.gravity(&moved_configuration(), &franka_hand(), &gravity);
    assert_close(with_hand.as_slice(), &[0., -28.2417, 0., 20.7531, 0.5095, 2.3446, 0.], GRAVITY_TOLERANCE);
}

#[test]
fn mass_at_zero_matches_reference() {
    let model = fr3_model();
    let expected = [
        0.132725, -0.0414226, 0.10001, 0.0120435, 0.0446215, -0.0022471, -0.000369021, -0.0414226,
        2.73892, -0.0437247, -1.15444, -0.0372567, 0.0137625, -0.00396178, 0.10001, -0.0437247,
        0.10001, 0.0120435, 0.0446215, -0.0022471, -0.000369021, 0.0120435, -1.15444, 0.0120435,
        0.644215, 0.0116149, -0.00714566, 0.00225227, 0.0446215, -0.0372567, 0.0446215, 0.0116149,
        0.0446215, -0.0022471, -0.000369021, -0.0022471, 0.0137625, -0.0022471, -0.00714566,
        -0.0022471, 0.0313282, 0.000174882, -0.000369021, -0.00396178, -0.000369021, 0.00225227,
        -0.000369021, 0.000174882, 0.000119426,
    ];
    let mass = model.mass(&JointVector::zeros(), &RigidBodyInertia::zero());
    assert_close(mass.as_slice(), &expected, EXACT_TOLERANCE);
}

#[test]
fn mass_at_zero_with_hand_matches_reference() {
    let model = fr3_model();
    let expected = [
        0.141436, -0.0414226, 0.108721, 0.0120435, 0.0533324, -0.0022471, -0.00278442, -0.0414226,
        2.97982, -0.0437247, -1.25956, -0.0372567, 0.0605572, -0.00396178, 0.108721, -0.0437247,
        0.108721, 0.0120435, 0.0533324, -0.0022471, -0.00278442, 0.0120435, -1.25956, 0.0120435,
        0.691427, 0.0116149, -0.0282393, 0.00225227, 0.0533324, -0.0372567, 0.0533324, 0.0116149,
        0.0533324, -0.0022471, -0.00278442, -0.0022471, 0.0605572, -0.0022471, -0.0282393,
        -0.0022471, 0.0545405, 0.000174882, -0.00278442, -0.00396178, -0.00278442, 0.00225227,
        -0.00278442, 0.000174882, 0.00189243,
    ];
    let mass = model.mass(&JointVector::zeros(), &franka_hand());
    assert_close(mass.as_slice(), &expected, EXACT_TOLERANCE);
}

#[test]
fn mass_at_default_configurations_matches_reference() {
    let model = fr3_model();
    let without_load = [
        0.9831, -0.0078, 0.9742, -0.0254, -0.0351, -0.0040, -0.0004, -0.0078, 1.5390, 0.0053,
        -0.6641, -0.0147, -0.0828, -0.0012, 0.9742, 0.0053, 0.9742, -0.0254, -0.0351, -0.0040,
        -0.0004, -0.0254, -0.6641, -0.0254, 0.8185, 0.0268, 0.0888, -0.0016, -0.0351, -0.0147,
        -0.0351, 0.0268, 0.0114, 0.0047, 0.0002, -0.0040, -0.0828, -0.0040, 0.0888, 0.0047,
        0.0202, 0.0003, -0.0004, -0.0012, -0.0004, -0.0016, 0.0002, 0.0003, 0.0002,
    ];
    let mass = model.mass(&default_configuration(), &RigidBodyInertia::zero());
    assert_close(mass.as_slice(), &without_load, MASS_TOLERANCE);
    let with_hand = [
        1.1749, -0.0085, 1.1660, -0.0247, -0.0261, -0.0033, -0.0057, -0.0085, 1.7281, 0.0046,
        -0.8304, -0.0142, -0.1209, -0.0005, 1.1660, 0.0046, 1.1660, -0.0247, -0.0261, -0.0033,
        -0.0057, -0.0247, -0.8304, -0.0247, 1.0399, 0.0263, 0.1526, -0.0023, -0.0261, -0.0142,
        -0.0261, 0.0263, 0.0112, 0.0042, 0.0012, -0.0033, -0.1209, -0.0033, 0.1526, 0.0042,
        0.0390, -0.0004, -0.0057, -0.0005, -0.0057, -0.0023, 0.0012, -0.0004, 0.0019,
    ];
    let mass = model.mass(&moved_configuration(), &franka_hand());
    assert_close(mass.as_slice(), &with_hand, MASS_TOLERANCE);
}

#[test]
fn coriolis_is_zero_at_rest() {
    let model = fr3_model();
    let coriolis = model.coriolis(&default_configuration(), &JointVector::zeros(), &franka_hand());
    assert_close(coriolis.as_slice(), &[0.0; NUM_JOINTS], 1e-12);
}

#[test]
fn coriolis_at_zero_with_hand_matches_reference() {
    let model = fr3_model();
    let coriolis = model.coriolis(&JointVector::zeros(), &JointVector::repeat(1.0), &franka_hand());
    assert_close(
        coriolis.as_slice(),
        &[0.25211, -1.97502, 0.254311, 0.987115, 0.122285, -0.256464, -0.000634562],
        EXACT_TOLERANCE,
    );
}

#[test]
fn coriolis_at_default_configurations_matches_reference() {
    let model = fr3_model();
    let unit_velocity = JointVector::repeat(1.0);
    let without_load = model.coriolis(&default_configuration(), &unit_velocity, &RigidBodyInertia::zero());
    assert_close(without_load.as_slice(), &[2.4562, -0.8199, 2.4532, -2.3450, -0.1492, -0.1801, 0.0164], CORIOLIS_TOLERANCE);
    let with_hand = model.coriolis(&moved_configuration(), &unit_velocity, &franka_hand());
    assert_close(with_hand.as_slice(), &[3.1468, -0.6225, 3.1439, -3.1063, -0.0658, -0.5416, 0.0089], CORIOLIS_TOLERANCE);
}

#[test]
fn mass_matrix_columns_match_inverse_dynamics() {
    let model = fr3_model();
    let q = moved_configuration();
    let dq = JointVector::from([0.3, -0.2, 0.5, 0.1, -0.4, 0.2, 0.6]);
    let gravity = Vector3::from(EARTH_GRAVITY);
    let payload = franka_hand();
    let bias = dynamics::inverse_dynamics(&model.chain, &payload, &q, &dq, &JointVector::zeros(), &gravity);
    let mass = model.mass(&q, &payload);
    for joint_index in 0..NUM_JOINTS {
        let mut unit_acceleration = JointVector::zeros();
        unit_acceleration[joint_index] = 1.0;
        let torques = dynamics::inverse_dynamics(&model.chain, &payload, &q, &dq, &unit_acceleration, &gravity);
        assert!((mass.column(joint_index) - (torques - bias)).norm() < 1e-10, "column {joint_index}");
    }
}

#[test]
fn gravity_torques_match_inverse_dynamics_at_rest() {
    let model = fr3_model();
    let q = moved_configuration();
    let gravity = Vector3::from(EARTH_GRAVITY);
    let payload = franka_hand();
    let at_rest = dynamics::inverse_dynamics(&model.chain, &payload, &q, &JointVector::zeros(), &JointVector::zeros(), &gravity);
    assert!((model.gravity(&q, &payload, &gravity) - at_rest).norm() < 1e-12);
}

#[test]
fn coriolis_equals_inverse_dynamics_minus_gravity() {
    let model = fr3_model();
    let q = moved_configuration();
    let dq = JointVector::from([0.3, -0.2, 0.5, 0.1, -0.4, 0.2, 0.6]);
    let gravity = Vector3::from(EARTH_GRAVITY);
    let payload = franka_hand();
    let two_pass = dynamics::inverse_dynamics(&model.chain, &payload, &q, &dq, &JointVector::zeros(), &gravity)
        - dynamics::gravity_torques(&model.chain, &payload, &q, &gravity);
    assert!((model.coriolis(&q, &dq, &payload) - two_pass).norm() < 1e-12);
}

/// Model whose outputs are simple functions of its inputs, so tests can tell which inputs it received.
struct EchoModel;

impl RobotModel for EchoModel {
    fn pose(&self, _frame: Frame, q: &JointVector, f_t_ee: &Isometry3<f64>, ee_t_k: &Isometry3<f64>) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(q[0], 0.0, 0.0), UnitQuaternion::identity()) * f_t_ee * ee_t_k
    }

    fn zero_jacobian(&self, _frame: Frame, q: &JointVector, _f_t_ee: &Isometry3<f64>, _ee_t_k: &Isometry3<f64>) -> Jacobian {
        Jacobian::from_element(q[1])
    }

    fn body_jacobian(&self, _frame: Frame, q: &JointVector, _f_t_ee: &Isometry3<f64>, _ee_t_k: &Isometry3<f64>) -> Jacobian {
        Jacobian::from_element(q[2])
    }

    fn mass(&self, _q: &JointVector, payload: &RigidBodyInertia) -> MassMatrix {
        MassMatrix::from_element(payload.mass)
    }

    fn coriolis(&self, _q: &JointVector, dq: &JointVector, _payload: &RigidBodyInertia) -> JointVector {
        *dq
    }

    fn gravity(&self, _q: &JointVector, _payload: &RigidBodyInertia, gravity: &Vector3<f64>) -> JointVector {
        JointVector::from_element(gravity.z)
    }
}

#[test]
fn from_state_methods_forward_state_fields() {
    let zeroed_bytes = [0u8; std::mem::size_of::<crate::wire::robot::RawRobotState>()];
    // SAFETY: the buffer is exactly the size of `RawRobotState`, whose fields are plain numbers.
    let mut state = unsafe { crate::wire::robot::RawRobotState::from_bytes(&zeroed_bytes) }.to_robot_state();
    state.q = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0];
    state.dq = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
    state.f_t_ee = Isometry3::<f64>::from_parts(Translation3::new(0.0, 0.5, 0.0), UnitQuaternion::identity()).to_homogeneous().as_slice().try_into().unwrap();
    state.ee_t_k = Isometry3::<f64>::from_parts(Translation3::new(0.0, 0.0, 0.25), UnitQuaternion::identity()).to_homogeneous().as_slice().try_into().unwrap();
    state.m_ee = 0.7;
    state.m_load = 0.3;
    state.o_ddp_o = [0.0, 0.0, -9.7];

    let model = EchoModel;
    assert!((model.pose_from_state(Frame::Stiffness, &state).translation.vector - Vector3::new(0.1, 0.5, 0.25)).norm() < 1e-12);
    assert_eq!(model.zero_jacobian_from_state(Frame::Flange, &state)[(0, 0)], 0.2);
    assert_eq!(model.body_jacobian_from_state(Frame::Flange, &state)[(0, 0)], 0.3);
    assert!((model.mass_from_state(&state)[(0, 0)] - 1.0).abs() < 1e-12);
    assert_eq!(model.coriolis_from_state(&state), JointVector::from(state.dq));
    assert_eq!(model.gravity_from_state(&state)[0], -9.7);
}

/// Maps the frame index emitted by validation/reference/model.cpp back to a `Frame`.
fn frame_from_index(index: usize) -> Frame {
    match index {
        0 => Frame::Joint1,
        1 => Frame::Joint2,
        2 => Frame::Joint3,
        3 => Frame::Joint4,
        4 => Frame::Joint5,
        5 => Frame::Joint6,
        6 => Frame::Joint7,
        7 => Frame::Flange,
        8 => Frame::EndEffector,
        9 => Frame::Stiffness,
        other => panic!("invalid frame index {other}"),
    }
}

/// Builds an isometry from a column-major 4x4 transform.
fn isometry_from_columns(columns: &[f64]) -> Isometry3<f64> {
    let matrix: [f64; 16] = columns.try_into().expect("16 values");
    crate::types::CartesianPose::from_column_major(&matrix).inner
}

/// Builds a payload from a mass, a 3-element centre of mass, and a 9-element inertia (about the
/// centre of mass), matching libfranka's `m_total`, `F_x_Ctotal`, and `I_total`.
fn payload_from(mass: f64, center_of_mass: &[f64], inertia: &[f64]) -> RigidBodyInertia {
    RigidBodyInertia::new(
        mass,
        Vector3::from_column_slice(center_of_mass),
        Matrix3::from_column_slice(inertia),
    )
}

/// Checks franka-rs's model outputs against libfranka's, both built from the same URDF.
///
/// Reads validation/data/model_cases.txt, produced by validation/reference/model.cpp. That file is
/// generated inside the devcontainer, where the Pinocchio-backed franka::Model builds; when it is
/// absent this test skips so the suite stays green. The tolerances account for nalgebra-vs-Pinocchio
/// differences on the same URDF and are initial values to tighten against the first generated data.
#[test]
fn model_matches_libfranka() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/validation/data/model_cases.txt");
    let Ok(text) = std::fs::read_to_string(path) else {
        eprintln!("skipping: {path} not present (generate it in the devcontainer via validation/reference/model.cpp)");
        return;
    };

    let urdf = include_str!("../../tests/fixtures/fr3_robot.urdf");
    let model = Model::from_urdf(urdf).unwrap();
    let kinematics_tolerance = 1e-6;
    let dynamics_tolerance = 1e-4;

    let mut cases = 0;
    for line in text.lines() {
        let mut fields = line.split(' ');
        let tag = fields.next().expect("tag");
        let v: Vec<f64> = fields.map(|field| field.parse().expect("number")).collect();

        match tag {
            "urdf_bytes" => {
                assert_eq!(
                    v[0] as usize,
                    urdf.len(),
                    "model_cases.txt was generated from a different URDF; regenerate it from tests/fixtures/fr3_robot.urdf"
                );
            }
            "pose" => {
                assert_eq!(v.len(), 56, "pose: field count");
                let q = JointVector::from_column_slice(&v[0..7]);
                let frame = frame_from_index(v[39] as usize);
                let actual = model
                    .pose(frame, &q, &isometry_from_columns(&v[7..23]), &isometry_from_columns(&v[23..39]))
                    .to_homogeneous();
                assert_close(actual.as_slice(), &v[40..56], kinematics_tolerance);
            }
            "zero_jacobian" => {
                assert_eq!(v.len(), 82, "zero_jacobian: field count");
                let q = JointVector::from_column_slice(&v[0..7]);
                let frame = frame_from_index(v[39] as usize);
                let actual =
                    model.zero_jacobian(frame, &q, &isometry_from_columns(&v[7..23]), &isometry_from_columns(&v[23..39]));
                assert_close(actual.as_slice(), &v[40..82], kinematics_tolerance);
            }
            "body_jacobian" => {
                assert_eq!(v.len(), 82, "body_jacobian: field count");
                let q = JointVector::from_column_slice(&v[0..7]);
                let frame = frame_from_index(v[39] as usize);
                let actual =
                    model.body_jacobian(frame, &q, &isometry_from_columns(&v[7..23]), &isometry_from_columns(&v[23..39]));
                assert_close(actual.as_slice(), &v[40..82], kinematics_tolerance);
            }
            "mass" => {
                assert_eq!(v.len(), 69, "mass: field count");
                let q = JointVector::from_column_slice(&v[0..7]);
                let load = payload_from(v[16], &v[17..20], &v[7..16]);
                assert_close(model.mass(&q, &load).as_slice(), &v[20..69], dynamics_tolerance);
            }
            "coriolis" => {
                assert_eq!(v.len(), 37, "coriolis: field count");
                let q = JointVector::from_column_slice(&v[0..7]);
                let dq = JointVector::from_column_slice(&v[7..14]);
                let load = payload_from(v[23], &v[24..27], &v[14..23]);
                assert_close(model.coriolis(&q, &dq, &load).as_slice(), &v[30..37], dynamics_tolerance);
            }
            "gravity" => {
                assert_eq!(v.len(), 21, "gravity: field count");
                let q = JointVector::from_column_slice(&v[0..7]);
                let load = payload_from(v[7], &v[8..11], &[0.0; 9]);
                let gravity = Vector3::from_column_slice(&v[11..14]);
                assert_close(model.gravity(&q, &load, &gravity).as_slice(), &v[14..21], dynamics_tolerance);
            }
            other => panic!("unknown tag {other}"),
        }
        cases += 1;
    }
    assert!(cases > 0, "no cases in {path}");
}
