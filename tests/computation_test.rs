//! Checks franka-rs's low-pass filter and rate-limiting functions against libfranka's outputs.
//!
//! Each line of validation/data/computation_cases.txt is `<tag> <inputs...> <outputs...>`, produced
//! by validation/reference/computation.cpp running the libfranka functions on the same inputs. This
//! test re-runs the franka-rs port on those inputs and compares against libfranka's outputs.

use franka_rs::lowpass_filter::{cartesian_lowpass_filter, lowpass_filter};
use franka_rs::rate_limiting::{
    limit_rate_cartesian_pose, limit_rate_cartesian_velocity, limit_rate_joint_positions,
    limit_rate_joint_velocities, limit_rate_position, limit_rate_torques, limit_rate_velocity,
};

/// Converts a slice into a fixed-size array, panicking if the length does not match.
fn arr<const N: usize>(values: &[f64]) -> [f64; N] {
    values.try_into().expect("slice length matches array")
}

/// Asserts `actual` and `expected` agree element-wise within `tolerance`.
fn assert_close(tag: &str, actual: &[f64], expected: &[f64], tolerance: f64) {
    assert_eq!(actual.len(), expected.len(), "{tag}: output length");
    for (index, (actual_value, expected_value)) in actual.iter().zip(expected).enumerate() {
        assert!(
            (actual_value - expected_value).abs() <= tolerance,
            "{tag}[{index}]: actual {actual_value}, expected {expected_value} (tolerance {tolerance})"
        );
    }
}

/// Tolerance for functions whose franka-rs and libfranka implementations share the same scalar or
/// vector arithmetic.
const EXACT: f64 = 1e-12;
/// Tolerance for functions that interpolate or extract rotations (quaternion slerp, axis-angle),
/// where nalgebra and Eigen can differ slightly. Chosen empirically; the near-180-degree cases are
/// the sensitive regime and may need loosening if the reference data is regenerated against a
/// different Eigen build.
const ROTATION: f64 = 1e-9;

#[test]
fn computation_matches_libfranka() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/validation/data/computation_cases.txt");
    let text = std::fs::read_to_string(path).unwrap_or_else(|error| {
        panic!(
            "cannot read {path}: {error}\n\
             regenerate it by building validation/reference/computation.cpp and running \
             `./computation > validation/data/computation_cases.txt`"
        )
    });

    let mut cases = 0;
    for line in text.lines() {
        let mut fields = line.split(' ');
        let tag = fields.next().expect("tag");
        let v: Vec<f64> = fields.map(|field| field.parse().expect("number")).collect();

        // Asserts the line has exactly the expected number of values before slicing, so a stray or
        // duplicated value cannot be silently dropped.
        let check_len = |expected: usize| assert_eq!(v.len(), expected, "{tag}: field count");

        match tag {
            "lowpass_filter" => {
                check_len(5);
                let out = lowpass_filter(v[0], v[1], v[2], v[3]);
                assert_close(tag, &[out], &v[4..5], EXACT);
            }
            "cartesian_lowpass_filter" => {
                check_len(50);
                let out = cartesian_lowpass_filter(v[0], &arr::<16>(&v[1..17]), &arr::<16>(&v[17..33]), v[33]);
                assert_close(tag, &out, &v[34..50], ROTATION);
            }
            "limit_rate_torques" => {
                check_len(28);
                let out = limit_rate_torques(&arr::<7>(&v[0..7]), &arr::<7>(&v[7..14]), &arr::<7>(&v[14..21]));
                assert_close(tag, &out, &v[21..28], EXACT);
            }
            "limit_rate_velocity" => {
                check_len(8);
                let out = limit_rate_velocity(v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
                assert_close(tag, &[out], &v[7..8], EXACT);
            }
            "limit_rate_position" => {
                check_len(9);
                let out = limit_rate_position(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
                assert_close(tag, &[out], &v[8..9], EXACT);
            }
            "limit_rate_joint_velocities" => {
                check_len(56);
                let out = limit_rate_joint_velocities(
                    &arr::<7>(&v[0..7]),
                    &arr::<7>(&v[7..14]),
                    &arr::<7>(&v[14..21]),
                    &arr::<7>(&v[21..28]),
                    &arr::<7>(&v[28..35]),
                    &arr::<7>(&v[35..42]),
                    &arr::<7>(&v[42..49]),
                );
                assert_close(tag, &out, &v[49..56], EXACT);
            }
            "limit_rate_joint_positions" => {
                check_len(63);
                let out = limit_rate_joint_positions(
                    &arr::<7>(&v[0..7]),
                    &arr::<7>(&v[7..14]),
                    &arr::<7>(&v[14..21]),
                    &arr::<7>(&v[21..28]),
                    &arr::<7>(&v[28..35]),
                    &arr::<7>(&v[35..42]),
                    &arr::<7>(&v[42..49]),
                    &arr::<7>(&v[49..56]),
                );
                assert_close(tag, &out, &v[56..63], EXACT);
            }
            "limit_rate_cartesian_velocity" => {
                check_len(30);
                let out = limit_rate_cartesian_velocity(
                    v[0], v[1], v[2], v[3], v[4], v[5],
                    &arr::<6>(&v[6..12]),
                    &arr::<6>(&v[12..18]),
                    &arr::<6>(&v[18..24]),
                );
                assert_close(tag, &out, &v[24..30], EXACT);
            }
            "limit_rate_cartesian_pose" => {
                check_len(66);
                let out = limit_rate_cartesian_pose(
                    v[0], v[1], v[2], v[3], v[4], v[5],
                    &arr::<16>(&v[6..22]),
                    &arr::<16>(&v[22..38]),
                    &arr::<6>(&v[38..44]),
                    &arr::<6>(&v[44..50]),
                );
                assert_close(tag, &out, &v[50..66], ROTATION);
            }
            other => panic!("unknown tag {other}"),
        }
        cases += 1;
    }
    assert!(cases > 0, "no cases found in {path}");
}
