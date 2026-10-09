use std::f64::consts::PI;

use crate::eigen_compat;
use crate::errors::{FrankaError, FrankaResult};

/// Maximum cutoff frequency in Hz.
pub const MAX_CUTOFF_FREQUENCY: f64 = 1000.0;

/// Default cutoff frequency in Hz.
pub const DEFAULT_CUTOFF_FREQUENCY: f64 = 100.0;

/// Returns an error unless the sample time is non-negative and finite and the cutoff frequency
/// positive and finite (libfranka's filter checks).
fn check_parameters(sample_time: f64, cutoff_frequency: f64) -> FrankaResult<()> {
    if sample_time < 0.0 || !sample_time.is_finite() {
        return Err(FrankaError::InvalidArgument {
            message: format!("lowpass filter: sample time is negative, infinite or NaN: {sample_time}"),
        });
    }
    if cutoff_frequency <= 0.0 || !cutoff_frequency.is_finite() {
        return Err(FrankaError::InvalidArgument {
            message: format!("lowpass filter: cutoff frequency is zero, negative, infinite or NaN: {cutoff_frequency}"),
        });
    }
    Ok(())
}

/// Returns an error unless the current and last signal values are finite.
fn check_signal(current: f64, last: f64) -> FrankaResult<()> {
    if !current.is_finite() || !last.is_finite() {
        return Err(FrankaError::InvalidArgument {
            message: "lowpass filter: current or past input value of the signal to be filtered is infinite or NaN".into(),
        });
    }
    Ok(())
}

/// Returns the filter gain for a sample time and cutoff frequency.
fn gain(sample_time: f64, cutoff_frequency: f64) -> f64 {
    sample_time / (sample_time + (1.0 / (2.0 * PI * cutoff_frequency)))
}

/// Applies a first-order low-pass filter to a scalar signal.
///
/// # Arguments
/// * `sample_time` - Sample time constant (e.g., 0.001 for 1kHz).
/// * `current` - Current value of the signal.
/// * `last` - Value of the signal at the previous time step.
/// * `cutoff_frequency` - Cutoff frequency of the filter in Hz.
///
/// # Errors
/// [`FrankaError::InvalidArgument`] for a negative or non-finite sample time, a cutoff frequency
/// that is not positive and finite, or a non-finite signal value.
pub fn lowpass_filter(sample_time: f64, current: f64, last: f64, cutoff_frequency: f64) -> FrankaResult<f64> {
    check_parameters(sample_time, cutoff_frequency)?;
    check_signal(current, last)?;
    let gain = gain(sample_time, cutoff_frequency);
    Ok(gain * current + (1.0 - gain) * last)
}

/// Applies a first-order low-pass filter to each joint of a joint-level array, as libfranka's
/// control loop filters each joint.
///
/// # Errors
/// As [`lowpass_filter`].
pub fn lowpass_filter_joints(
    sample_time: f64,
    current: &[f64; 7],
    last: &[f64; 7],
    cutoff_frequency: f64,
) -> FrankaResult<[f64; 7]> {
    let mut result = [0.0; 7];
    for i in 0..7 {
        result[i] = lowpass_filter(sample_time, current[i], last[i], cutoff_frequency)?;
    }
    Ok(result)
}

/// Applies a first-order low-pass filter to a Cartesian transformation matrix, with libfranka's
/// arithmetic: the translation is filtered linearly; the rotations, taken as Eigen's
/// `Affine3d::rotation()`, are interpolated by quaternion slerp and normalized.
///
/// Both matrices are column-major 4x4 homogeneous transforms; the result keeps `current`'s last
/// row.
///
/// # Errors
/// As [`lowpass_filter`], for any of the 16 values of either matrix.
pub fn cartesian_lowpass_filter(
    sample_time: f64,
    current: &[f64; 16],
    last: &[f64; 16],
    cutoff_frequency: f64,
) -> FrankaResult<[f64; 16]> {
    check_parameters(sample_time, cutoff_frequency)?;
    for (current_value, last_value) in current.iter().zip(last) {
        check_signal(*current_value, *last_value)?;
    }
    let gain = gain(sample_time, cutoff_frequency);
    let mut result = *current;
    for i in 12..15 {
        result[i] = gain * current[i] + (1.0 - gain) * last[i];
    }
    let orientation = eigen_compat::quaternion_from_matrix(&eigen_compat::affine_rotation(&eigen_compat::linear_part(current)));
    let orientation_last = eigen_compat::quaternion_from_matrix(&eigen_compat::affine_rotation(&eigen_compat::linear_part(last)));
    let filtered = eigen_compat::quaternion_slerp(&orientation_last, gain, &orientation);
    let rotation = eigen_compat::quaternion_to_matrix(&eigen_compat::quaternion_normalized(&filtered));
    for column in 0..3 {
        for row in 0..3 {
            result[column * 4 + row] = rotation[row][column];
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lowpass_filter_passthrough_at_high_frequency() {
        // With very high cutoff, gain ≈ 1, so output ≈ current
        let result = lowpass_filter(0.001, 10.0, 5.0, 100000.0).unwrap();
        assert!((result - 10.0).abs() < 0.01);
    }

    #[test]
    fn lowpass_filter_holds_at_low_frequency() {
        // With very low cutoff, gain ≈ 0, so output ≈ last
        let result = lowpass_filter(0.001, 10.0, 5.0, 0.001).unwrap();
        assert!((result - 5.0).abs() < 0.01);
    }

    #[test]
    fn lowpass_filter_default_frequency() {
        let sample_time = 0.001;
        let cutoff = DEFAULT_CUTOFF_FREQUENCY;
        let gain = sample_time / (sample_time + 1.0 / (2.0 * PI * cutoff));

        let result = lowpass_filter(sample_time, 10.0, 5.0, cutoff).unwrap();
        let expected = gain * 10.0 + (1.0 - gain) * 5.0;
        assert!((result - expected).abs() < 1e-12);
    }

    #[test]
    fn cartesian_lowpass_filter_identity() {
        let identity = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let result = cartesian_lowpass_filter(0.001, &identity, &identity, 100.0).unwrap();
        for i in 0..16 {
            assert!(
                (result[i] - identity[i]).abs() < 1e-10,
                "mismatch at index {i}: {} vs {}",
                result[i],
                identity[i]
            );
        }
    }

    #[test]
    fn cartesian_lowpass_filter_translation_only() {
        let last = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let current = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0,
        ];

        let sample_time = 0.001;
        let cutoff = 100.0;
        let gain = sample_time / (sample_time + 1.0 / (2.0 * PI * cutoff));

        let result = cartesian_lowpass_filter(sample_time, &current, &last, cutoff).unwrap();

        // Translation x should be filtered: gain * 1.0 + (1-gain) * 0.0
        assert!((result[12] - gain).abs() < 1e-10);
        // Rotation should remain identity
        assert!((result[0] - 1.0).abs() < 1e-10);
        assert!((result[5] - 1.0).abs() < 1e-10);
        assert!((result[10] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn filters_reject_invalid_parameters_and_non_finite_signals() {
        let invalid = |result: FrankaResult<()>| matches!(result, Err(FrankaError::InvalidArgument { .. }));
        assert!(invalid(lowpass_filter(-0.001, 1.0, 0.0, 100.0).map(|_| ())));
        assert!(invalid(lowpass_filter(0.001, 1.0, 0.0, 0.0).map(|_| ())));
        assert!(invalid(lowpass_filter(0.001, 1.0, 0.0, f64::INFINITY).map(|_| ())));
        assert!(invalid(lowpass_filter(0.001, f64::NAN, 0.0, 100.0).map(|_| ())));
        assert!(invalid(lowpass_filter(0.001, 1.0, f64::INFINITY, 100.0).map(|_| ())));
        let mut joints = [0.0; 7];
        joints[6] = f64::NAN;
        assert!(invalid(lowpass_filter_joints(0.001, &joints, &[0.0; 7], 100.0).map(|_| ())));
        let mut identity = [0.0; 16];
        for index in [0, 5, 10, 15] {
            identity[index] = 1.0;
        }
        let mut bad = identity;
        bad[12] = f64::INFINITY;
        assert!(invalid(cartesian_lowpass_filter(0.001, &identity, &bad, 100.0).map(|_| ())));
        assert!(invalid(cartesian_lowpass_filter(0.001, &identity, &identity, -1.0).map(|_| ())));
    }
}
