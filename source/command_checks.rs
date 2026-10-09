//! Validity checks for command values, as libfranka's `control_tools.h` (`checkFinite`,
//! `checkMatrix`, `checkElbow`). Every rejection is a [`FrankaError::InvalidArgument`].

use crate::errors::{FrankaError, FrankaResult};

/// Returns an error unless every element of `values` is finite.
pub(crate) fn check_finite<const N: usize>(values: &[f64; N]) -> FrankaResult<()> {
    match values.iter().position(|value| !value.is_finite()) {
        Some(index) => Err(FrankaError::InvalidArgument {
            message: format!("element {index} is not finite: {}", values[index]),
        }),
        None => Ok(()),
    }
}

/// Returns an error unless `transform` is finite and a homogeneous transformation (column major):
/// last row exactly (0, 0, 0, 1), and every rotation column and row of unit length within 1e-5.
pub(crate) fn check_matrix(transform: &[f64; 16]) -> FrankaResult<()> {
    const ORTHONORMAL_THRESHOLD: f64 = 1e-5;
    check_finite(transform)?;
    let element = |row: usize, column: usize| transform[column * 4 + row];
    let unit = |a: f64, b: f64, c: f64| ((a.powi(2) + b.powi(2) + c.powi(2)).sqrt() - 1.0).abs() <= ORTHONORMAL_THRESHOLD;
    let homogeneous = element(3, 0) == 0.0
        && element(3, 1) == 0.0
        && element(3, 2) == 0.0
        && element(3, 3) == 1.0
        && (0..3).all(|column| unit(element(0, column), element(1, column), element(2, column)))
        && (0..3).all(|row| unit(element(row, 0), element(row, 1), element(row, 2)));
    if !homogeneous {
        return Err(FrankaError::InvalidArgument {
            message: "not a homogeneous transformation (has to be column major)".into(),
        });
    }
    Ok(())
}

/// Returns an error unless `elbow` is finite and its second element (the sign of joint 4) is
/// exactly +1 or -1.
pub(crate) fn check_elbow(elbow: &[f64; 2]) -> FrankaResult<()> {
    check_finite(elbow)?;
    if elbow[1] != 1.0 && elbow[1] != -1.0 {
        return Err(FrankaError::InvalidArgument {
            message: format!("invalid elbow: the sign of joint 4 must be +1 or -1, got {}", elbow[1]),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Column-major identity transformation.
    const IDENTITY: [f64; 16] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

    #[test]
    fn check_matrix_accepts_identity_and_rejects_non_homogeneous_matrices() {
        assert!(check_matrix(&IDENTITY).is_ok());
        let mut scaled = IDENTITY;
        scaled[0] = 1.001;
        assert!(matches!(check_matrix(&scaled), Err(FrankaError::InvalidArgument { .. })));
        let mut bottom_row = IDENTITY;
        bottom_row[3] = 0.1;
        assert!(matches!(check_matrix(&bottom_row), Err(FrankaError::InvalidArgument { .. })));
        let mut not_finite = IDENTITY;
        not_finite[12] = f64::NAN;
        assert!(matches!(check_matrix(&not_finite), Err(FrankaError::InvalidArgument { .. })));
    }

    #[test]
    fn check_elbow_requires_a_joint_4_sign_of_plus_or_minus_one() {
        assert!(check_elbow(&[0.3, -1.0]).is_ok());
        assert!(check_elbow(&[0.3, 1.0]).is_ok());
        assert!(matches!(check_elbow(&[0.3, 0.5]), Err(FrankaError::InvalidArgument { .. })));
        assert!(matches!(check_elbow(&[f64::INFINITY, 1.0]), Err(FrankaError::InvalidArgument { .. })));
    }
}
