//! Tests of [`Model`] against reference values computed for the FR3 URDF.

use std::path::Path;

use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};

use super::*;

/// Earth gravity in the base frame.
const EARTH_GRAVITY: [f64; 3] = [0.0, 0.0, -9.81];

/// Returns the model of the FR3 test URDF.
fn fr3_model() -> Model {
    Model::from_urdf(include_str!("../../tests/fixtures/fr3_robot.urdf")).unwrap()
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
fn coriolis_is_zero_at_rest() {
    let model = fr3_model();
    let coriolis = model.coriolis(&default_configuration(), &JointVector::zeros(), &franka_hand());
    assert_close(coriolis.as_slice(), &[0.0; NUM_JOINTS], 1e-12);
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

/// Configurations, velocities and payloads for the Coriolis matrix property tests.
fn coriolis_cases() -> Vec<(JointVector, JointVector, RigidBodyInertia)> {
    vec![
        (JointVector::zeros(), JointVector::repeat(1.0), franka_hand()),
        (moved_configuration(), JointVector::from([0.3, -0.2, 0.5, 0.1, -0.4, 0.2, 0.6]), RigidBodyInertia::zero()),
        (
            JointVector::from([0.5, -0.8, 0.3, -2.0, 0.7, 1.9, -0.4]),
            JointVector::from([-1.2, 0.8, 1.5, -0.6, 2.0, -1.7, 0.9]),
            franka_hand(),
        ),
    ]
}

#[test]
fn coriolis_matrix_times_velocity_equals_coriolis_vector() {
    let model = fr3_model();
    for (q, dq, payload) in coriolis_cases() {
        let product = model.coriolis_matrix(&q, &dq, &payload) * dq;
        let vector = model.coriolis(&q, &dq, &payload);
        assert!((product - vector).norm() < 1e-10, "C·dq {product} vs coriolis {vector}");
    }
}

#[test]
fn coriolis_matrix_is_zero_at_rest() {
    let model = fr3_model();
    let matrix = model.coriolis_matrix(&moved_configuration(), &JointVector::zeros(), &franka_hand());
    assert_eq!(matrix.norm(), 0.0);
}

#[test]
fn mass_derivative_minus_twice_coriolis_matrix_is_skew_symmetric() {
    // Ṁ along dq by a central difference; Ṁ − 2C must be skew-symmetric for this C.
    let model = fr3_model();
    let step = 1e-6;
    for (q, dq, payload) in coriolis_cases() {
        let mass_derivative = (model.mass(&(q + dq * step), &payload) - model.mass(&(q - dq * step), &payload)) / (2.0 * step);
        let n = mass_derivative - 2.0 * model.coriolis_matrix(&q, &dq, &payload);
        assert!((n + n.transpose()).norm() < 1e-7, "Ṁ − 2C not skew-symmetric: {}", (n + n.transpose()).norm());
    }
}

/// The mass matrix inverse is symmetric up to rounding, relative to its size.
#[test]
fn mass_matrix_inverse_is_symmetric() {
    let model = fr3_model();
    for (q, _, payload) in coriolis_cases() {
        // Entries (i, j) and (j, i) are computed separately, so rounding leaves a small asymmetry
        // that grows with the size of the inverse.
        let inverse = model.mass_matrix_inverse(&q, &payload);
        let relative_asymmetry = (inverse - inverse.transpose()).norm() / inverse.norm();
        assert!(relative_asymmetry < 1e-13, "relative asymmetry {relative_asymmetry:e}");
    }
}

/// Asserts that `model`'s mass matrix inverse is both a left and a right inverse of its mass matrix.
fn assert_mass_matrix_inverse_inverts_mass(model: &Model, q: &JointVector, payload: &RigidBodyInertia) {
    let mass = model.mass(q, payload);
    let inverse = model.mass_matrix_inverse(q, payload);
    let right = (mass * inverse - MassMatrix::identity()).norm();
    let left = (inverse * mass - MassMatrix::identity()).norm();
    assert!(right < 1e-10, "M·M⁻¹ differs from identity by {right}");
    assert!(left < 1e-10, "M⁻¹·M differs from identity by {left}");
}

/// The mass matrix inverse is a left and right inverse of the mass matrix on the FR3 model.
#[test]
fn mass_matrix_inverse_times_mass_is_identity() {
    let model = fr3_model();
    for (q, _, payload) in coriolis_cases() {
        assert_mass_matrix_inverse_inverts_mass(&model, &q, &payload);
    }
}

/// The mass matrix inverse stays correct when the joint axes are tilted or reversed, which
/// exercises the alignment of each axis with z.
#[test]
fn mass_matrix_inverse_with_rotated_axis_is_inverse() {
    let urdf = include_str!("../../tests/fixtures/fr3_robot.urdf");
    let z_axis = r#"<axis xyz="0 0 1"/>"#;
    assert_eq!(urdf.matches(z_axis).count(), NUM_JOINTS, "every joint axis must be replaced");
    for axis in ["0.6 0 0.8", "0 0 -1"] {
        let rotated = urdf.replace(z_axis, &format!(r#"<axis xyz="{axis}"/>"#));
        let model = Model::from_urdf(&rotated).unwrap();
        for (q, _, payload) in coriolis_cases() {
            assert_mass_matrix_inverse_inverts_mass(&model, &q, &payload);
        }
    }
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

    /// Returns a matrix filled with `q[0]` plus the payload mass, distinct from [`EchoModel::mass`].
    fn mass_matrix_inverse(&self, q: &JointVector, payload: &RigidBodyInertia) -> MassMatrix {
        MassMatrix::from_element(q[0] + payload.mass)
    }

    fn coriolis(&self, _q: &JointVector, dq: &JointVector, _payload: &RigidBodyInertia) -> JointVector {
        *dq
    }

    fn coriolis_matrix(&self, q: &JointVector, dq: &JointVector, payload: &RigidBodyInertia) -> CoriolisMatrix {
        CoriolisMatrix::from_diagonal(dq) * (q[0] + payload.mass)
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
    assert!((model.mass_matrix_inverse_from_state(&state)[(0, 0)] - 1.1).abs() < 1e-12);
    assert_eq!(model.coriolis_from_state(&state), JointVector::from(state.dq));
    assert!((model.coriolis_matrix_from_state(&state) - CoriolisMatrix::from_diagonal(&JointVector::from(state.dq)) * 1.1).norm() < 1e-12);
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
    crate::types::isometry_from_column_major(&matrix)
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
/// Reads the fixture tests/fixtures/model_cases.txt, produced by validation/reference/model.cpp
/// inside the devcontainer, where the Pinocchio-backed franka::Model builds. The test fails if the
/// fixture is missing or empty. The tolerances account for nalgebra-vs-Pinocchio differences on the
/// same URDF and are initial values to tighten against the first generated data.
#[test]
fn model_matches_libfranka() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/model_cases.txt");
    let text = std::fs::read_to_string(path).unwrap_or_else(|error| {
        panic!("cannot read {path} ({error}); generate it in the devcontainer via validation/reference/model.cpp")
    });
    let urdf_path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests").join("fixtures").join("fr3_robot.urdf");
    let urdf = std::fs::read_to_string(&urdf_path).unwrap();
    let model = Model::from_urdf(&urdf).unwrap();
    let kinematics_tolerance = 1e-6;
    let dynamics_tolerance = 1e-4;

    let mut cases = 0;
    let mut largest_coriolis_matrix_difference: f64 = 0.0;
    let mut largest_mass_matrix_inverse_difference: f64 = 0.0;
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
            "coriolis_matrix" => {
                assert_eq!(v.len(), 76, "coriolis_matrix: field count");
                let q = JointVector::from_column_slice(&v[0..7]);
                let dq = JointVector::from_column_slice(&v[7..14]);
                let load = payload_from(v[23], &v[24..27], &v[14..23]);
                let actual = model.coriolis_matrix(&q, &dq, &load);
                let largest = actual.iter().zip(&v[27..76]).map(|(a, e)| (a - e).abs()).fold(0.0, f64::max);
                largest_coriolis_matrix_difference = largest_coriolis_matrix_difference.max(largest);
                // Both evaluate the same algorithm, so agreement is at rounding level.
                assert_close(actual.as_slice(), &v[27..76], 1e-12);
            }
            "mass_matrix_inverse" => {
                assert_eq!(v.len(), 69, "mass_matrix_inverse: field count");
                let q = JointVector::from_column_slice(&v[0..7]);
                let load = payload_from(v[16], &v[17..20], &v[7..16]);
                let actual = model.mass_matrix_inverse(&q, &load);
                let expected = MassMatrix::from_column_slice(&v[20..69]);
                // Entries grow into the thousands for light distal links, so the bound is relative.
                let relative = (actual - expected).norm() / expected.norm();
                largest_mass_matrix_inverse_difference = largest_mass_matrix_inverse_difference.max(relative);
                assert!(relative < 1e-12, "mass_matrix_inverse: relative difference {relative:e}");
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
    eprintln!("largest |franka-rs - pinocchio| Coriolis matrix element: {largest_coriolis_matrix_difference:e}");
    eprintln!("largest |franka-rs - pinocchio| mass matrix inverse (relative norm): {largest_mass_matrix_inverse_difference:e}");
}
