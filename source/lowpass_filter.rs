use std::f64::consts::PI;

use nalgebra::{Matrix4, UnitQuaternion, Vector3};

/// Maximum cutoff frequency in Hz.
pub const MAX_CUTOFF_FREQUENCY: f64 = 1000.0;

/// Default cutoff frequency in Hz.
pub const DEFAULT_CUTOFF_FREQUENCY: f64 = 100.0;

/// Applies a first-order low-pass filter to a scalar signal.
///
/// # Arguments
/// * `sample_time` - Sample time constant (e.g., 0.001 for 1kHz).
/// * `current` - Current value of the signal.
/// * `last` - Value of the signal at the previous time step.
/// * `cutoff_frequency` - Cutoff frequency of the filter in Hz.
pub fn lowpass_filter(sample_time: f64, current: f64, last: f64, cutoff_frequency: f64) -> f64 {
    let gain = sample_time / (sample_time + (1.0 / (2.0 * PI * cutoff_frequency)));
    gain * current + (1.0 - gain) * last
}

/// Applies a first-order low-pass filter to a joint-level array.
pub fn lowpass_filter_joints(
    sample_time: f64,
    current: &[f64; 7],
    last: &[f64; 7],
    cutoff_frequency: f64,
) -> [f64; 7] {
    let gain = sample_time / (sample_time + (1.0 / (2.0 * PI * cutoff_frequency)));
    let mut result = [0.0; 7];
    for i in 0..7 {
        result[i] = gain * current[i] + (1.0 - gain) * last[i];
    }
    result
}

/// Applies a first-order low-pass filter to a Cartesian transformation matrix.
///
/// Translation components are filtered linearly. Rotation is interpolated
/// using spherical linear interpolation (SLERP).
///
/// Both matrices are column-major 4x4 homogeneous transforms.
pub fn cartesian_lowpass_filter(
    sample_time: f64,
    current: &[f64; 16],
    last: &[f64; 16],
    cutoff_frequency: f64,
) -> [f64; 16] {
    let gain = sample_time / (sample_time + (1.0 / (2.0 * PI * cutoff_frequency)));

    let current_mat = Matrix4::from_column_slice(current);
    let last_mat = Matrix4::from_column_slice(last);

    // Extract translations
    let current_translation = Vector3::new(current_mat[(0, 3)], current_mat[(1, 3)], current_mat[(2, 3)]);
    let last_translation = Vector3::new(last_mat[(0, 3)], last_mat[(1, 3)], last_mat[(2, 3)]);

    // Filter translation linearly
    let filtered_translation = gain * current_translation + (1.0 - gain) * last_translation;

    // Extract rotations as quaternions and SLERP
    let current_rot = current_mat.fixed_view::<3, 3>(0, 0).into_owned();
    let last_rot = last_mat.fixed_view::<3, 3>(0, 0).into_owned();

    let current_quat = UnitQuaternion::from_matrix(&current_rot);
    let last_quat = UnitQuaternion::from_matrix(&last_rot);

    let filtered_quat = last_quat.slerp(&current_quat, gain);

    // Reconstruct the filtered transformation matrix
    let mut result_mat = Matrix4::identity();
    result_mat
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&filtered_quat.to_rotation_matrix().into_inner());
    result_mat[(0, 3)] = filtered_translation.x;
    result_mat[(1, 3)] = filtered_translation.y;
    result_mat[(2, 3)] = filtered_translation.z;

    let mut result = [0.0; 16];
    result.copy_from_slice(result_mat.as_slice());
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lowpass_filter_passthrough_at_high_frequency() {
        // With very high cutoff, gain ≈ 1, so output ≈ current
        let result = lowpass_filter(0.001, 10.0, 5.0, 100000.0);
        assert!((result - 10.0).abs() < 0.01);
    }

    #[test]
    fn lowpass_filter_holds_at_low_frequency() {
        // With very low cutoff, gain ≈ 0, so output ≈ last
        let result = lowpass_filter(0.001, 10.0, 5.0, 0.001);
        assert!((result - 5.0).abs() < 0.01);
    }

    #[test]
    fn lowpass_filter_default_frequency() {
        let sample_time = 0.001;
        let cutoff = DEFAULT_CUTOFF_FREQUENCY;
        let gain = sample_time / (sample_time + 1.0 / (2.0 * PI * cutoff));

        let result = lowpass_filter(sample_time, 10.0, 5.0, cutoff);
        let expected = gain * 10.0 + (1.0 - gain) * 5.0;
        assert!((result - expected).abs() < 1e-12);
    }

    #[test]
    fn cartesian_lowpass_filter_identity() {
        let identity = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ];
        let result = cartesian_lowpass_filter(0.001, &identity, &identity, 100.0);
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

        let result = cartesian_lowpass_filter(sample_time, &current, &last, cutoff);

        // Translation x should be filtered: gain * 1.0 + (1-gain) * 0.0
        assert!((result[12] - gain).abs() < 1e-10);
        // Rotation should remain identity
        assert!((result[0] - 1.0).abs() < 1e-10);
        assert!((result[5] - 1.0).abs() < 1e-10);
        assert!((result[10] - 1.0).abs() < 1e-10);
    }
}
