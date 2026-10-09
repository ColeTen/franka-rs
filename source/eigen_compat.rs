//! 3-D rotation arithmetic reproducing, operation for operation, what Eigen 3.4 computes in
//! libfranka's Cartesian low-pass filter and Cartesian pose rate limiter, so that franka-rs's
//! results are bit-identical to libfranka's.
//!
//! The summation orders are not specified by Eigen; they are what GCC 11.4 generates for Eigen 3.4
//! with libfranka's flags (`-O3`, x86-64 SSE2, no FMA), and `sin`/`cos`/`acos`/`atan2` come from
//! the host's glibc (2.35 here) on both sides. A libfranka built with another compiler, `-march` or
//! FMA can round differently; `tests/computation_test.rs` checks against the libfranka in use.
//! Measured against Eigen on random inputs: 3×3 matrix products sum rows 0
//! and 1 sequentially (vectorized two rows at a time) and row 2 with the last two terms first;
//! 3-vector norms and dot products sum sequentially; the trace sums the last two terms first;
//! quaternion dot products sum coefficient pairs (x, z) and (y, w) first.

/// A 3×3 matrix, indexed `[row][column]`.
pub(crate) type Mat3 = [[f64; 3]; 3];

/// A quaternion as Eigen stores its coefficients: `[x, y, z, w]`.
pub(crate) type Quat = [f64; 4];

/// Returns the 3×3 identity.
pub(crate) fn identity() -> Mat3 {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

/// Returns the top-left 3×3 block of a column-major 4×4 matrix.
pub(crate) fn linear_part(transform: &[f64; 16]) -> Mat3 {
    std::array::from_fn(|row| std::array::from_fn(|column| transform[column * 4 + row]))
}

/// Returns the Euclidean norm of a 3-vector, summed as Eigen's `norm()`.
pub(crate) fn norm3(v: &[f64; 3]) -> f64 {
    squared_norm3(v).sqrt()
}

/// Returns the squared norm of a 3-vector, summed as Eigen's `squaredNorm()`.
pub(crate) fn squared_norm3(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]
}

/// Returns the dot product of two 3-vectors, summed as Eigen's `dot()` and `aᵀ * b`.
pub(crate) fn dot3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}

/// Returns the sum of a product row's three terms in Eigen's order for result row `row`: rows 0
/// and 1 sequentially, row 2 with the last two terms first.
fn product_sum(row: usize, terms: [f64; 3]) -> f64 {
    if row < 2 { (terms[0] + terms[1]) + terms[2] } else { terms[0] + (terms[1] + terms[2]) }
}

/// Returns `a * b`, summed as Eigen's 3×3 product.
pub(crate) fn mul(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| product_sum(i, std::array::from_fn(|k| a[i][k] * b[k][j]))))
}

/// Returns `scalar * (a * b)` as Eigen evaluates it: the scalar folded into the left operand.
pub(crate) fn scaled_mul(scalar: f64, a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| product_sum(i, std::array::from_fn(|k| (scalar * a[i][k]) * b[k][j]))))
}

/// Returns `a * bᵀ`, summed as Eigen's 3×3 product.
pub(crate) fn mul_transpose(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| product_sum(i, std::array::from_fn(|k| a[i][k] * b[j][k]))))
}

/// Returns the determinant of a 3×3 matrix by Eigen's cofactor formula.
fn determinant(m: &Mat3) -> f64 {
    let helper = |a: usize, b: usize, c: usize| m[0][a] * (m[1][b] * m[2][c] - m[1][c] * m[2][b]);
    helper(0, 1, 2) - helper(1, 0, 2) + helper(2, 0, 1)
}

/// A plane rotation `(c, s)` as Eigen's `JacobiRotation`.
#[derive(Clone, Copy)]
struct Jacobi {
    c: f64,
    s: f64,
}

impl Jacobi {
    /// Returns the concatenation `self * other`.
    fn then(self, other: Jacobi) -> Jacobi {
        Jacobi { c: self.c * other.c - self.s * other.s, s: self.c * other.s + self.s * other.c }
    }

    /// Returns the transposed rotation.
    fn transpose(self) -> Jacobi {
        Jacobi { c: self.c, s: -self.s }
    }

    /// Returns the rotation that diagonalizes the symmetric 2×2 block `[[x, y], [y, z]]`
    /// (Eigen's `makeJacobi`).
    fn make_jacobi(x: f64, y: f64, z: f64) -> Jacobi {
        let deno = 2.0 * y.abs();
        if deno < f64::MIN_POSITIVE {
            return Jacobi { c: 1.0, s: 0.0 };
        }
        let tau = (x - z) / deno;
        let w = (tau * tau + 1.0).sqrt();
        let t = if tau > 0.0 { 1.0 / (tau + w) } else { 1.0 / (tau - w) };
        let sign_t = if t > 0.0 { 1.0 } else { -1.0 };
        let n = 1.0 / (t * t + 1.0).sqrt();
        Jacobi { c: n, s: ((-sign_t * (y / y.abs())) * t.abs()) * n }
    }
}

/// Applies the rotation to the pair `(x, y)` element by element (Eigen's
/// `apply_rotation_in_the_plane`), skipping the identity rotation.
fn rotate_pair(x: &mut [f64; 3], y: &mut [f64; 3], j: Jacobi) {
    if j.c == 1.0 && j.s == 0.0 {
        return;
    }
    for index in 0..3 {
        let (xi, yi) = (x[index], y[index]);
        x[index] = j.c * xi + j.s * yi;
        y[index] = -j.s * xi + j.c * yi;
    }
}

/// Applies `j` to rows `p` and `q` of `m` (Eigen's `applyOnTheLeft`).
fn apply_on_the_left(m: &mut Mat3, p: usize, q: usize, j: Jacobi) {
    let (mut row_p, mut row_q) = (m[p], m[q]);
    rotate_pair(&mut row_p, &mut row_q, j);
    m[p] = row_p;
    m[q] = row_q;
}

/// Applies `j` to columns `p` and `q` of `m` (Eigen's `applyOnTheRight`).
fn apply_on_the_right(m: &mut Mat3, p: usize, q: usize, j: Jacobi) {
    let mut column_p = [m[0][p], m[1][p], m[2][p]];
    let mut column_q = [m[0][q], m[1][q], m[2][q]];
    rotate_pair(&mut column_p, &mut column_q, j.transpose());
    for row in 0..3 {
        m[row][p] = column_p[row];
        m[row][q] = column_q[row];
    }
}

/// Returns the left and right rotations that diagonalize the 2×2 block `(p, q)` of `m` (Eigen's
/// `real_2x2_jacobi_svd`).
fn real_2x2_jacobi_svd(m: &Mat3, p: usize, q: usize) -> (Jacobi, Jacobi) {
    let mut block = [[m[p][p], m[p][q], 0.0], [m[q][p], m[q][q], 0.0], [0.0; 3]];
    let t = block[0][0] + block[1][1];
    let d = block[1][0] - block[0][1];
    let rot1 = if d.abs() < f64::MIN_POSITIVE {
        Jacobi { c: 1.0, s: 0.0 }
    } else {
        let u = t / d;
        let tmp = (1.0 + u * u).sqrt();
        Jacobi { c: u / tmp, s: 1.0 / tmp }
    };
    apply_on_the_left(&mut block, 0, 1, rot1);
    let j_right = Jacobi::make_jacobi(block[0][0], block[0][1], block[1][1]);
    (rot1.then(j_right.transpose()), j_right)
}

/// Returns the larger of two values as Eigen's `numext::maxi` (`b` only if `a < b`).
fn maxi(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// Returns the rotation part of an affine transformation's linear part `a`, as Eigen's
/// `Transform<double, 3, Affine>::rotation()`: the polar factor `U·Vᵀ` of a two-sided Jacobi SVD,
/// with the last column of `U` negated when `U·Vᵀ` is a reflection.
pub(crate) fn affine_rotation(a: &Mat3) -> Mat3 {
    // Eigen's JacobiSVD::compute for a square real 3×3 matrix with full U and V.
    let precision = 2.0 * f64::EPSILON;
    let consider_as_zero = f64::MIN_POSITIVE;
    let mut scale = a.iter().flatten().fold(0.0_f64, |largest, value| maxi(largest, value.abs()));
    if scale == 0.0 {
        scale = 1.0;
    }
    let mut work: Mat3 = std::array::from_fn(|i| std::array::from_fn(|j| a[i][j] / scale));
    let mut u = identity();
    let mut v = identity();
    let mut max_diag_entry = maxi(maxi(work[0][0].abs(), work[1][1].abs()), work[2][2].abs());

    let mut finished = false;
    while !finished {
        finished = true;
        for p in 1..3 {
            for q in 0..p {
                let threshold = maxi(consider_as_zero, precision * max_diag_entry);
                if work[p][q].abs() > threshold || work[q][p].abs() > threshold {
                    finished = false;
                    let (j_left, j_right) = real_2x2_jacobi_svd(&work, p, q);
                    apply_on_the_left(&mut work, p, q, j_left);
                    apply_on_the_right(&mut u, p, q, j_left.transpose());
                    apply_on_the_right(&mut work, p, q, j_right);
                    apply_on_the_right(&mut v, p, q, j_right);
                    max_diag_entry = maxi(max_diag_entry, maxi(work[p][p].abs(), work[q][q].abs()));
                }
            }
        }
    }

    // Singular values made non-negative (negating U's column), scaled back, sorted descending.
    let mut singular_values = [0.0; 3];
    for i in 0..3 {
        let value = work[i][i];
        singular_values[i] = value.abs();
        if value < 0.0 {
            for row in u.iter_mut() {
                row[i] = -row[i];
            }
        }
    }
    for value in singular_values.iter_mut() {
        *value *= scale;
    }
    for i in 0..3 {
        let mut position = i;
        for candidate in i + 1..3 {
            if singular_values[candidate] > singular_values[position] {
                position = candidate;
            }
        }
        if singular_values[position] == 0.0 {
            break;
        }
        if position != i {
            singular_values.swap(i, position);
            for row in u.iter_mut().chain(v.iter_mut()) {
                row.swap(i, position);
            }
        }
    }

    // Eigen's computeRotationScaling.
    let x = if determinant(&mul_transpose(&u, &v)) < 0.0 { -1.0 } else { 1.0 };
    for row in u.iter_mut() {
        row[2] *= x;
    }
    mul_transpose(&u, &v)
}

/// Returns the quaternion of a rotation matrix (Eigen's Shoemake conversion).
pub(crate) fn quaternion_from_matrix(m: &Mat3) -> Quat {
    let mut q = [0.0; 4];
    let trace = m[0][0] + (m[1][1] + m[2][2]);
    if trace > 0.0 {
        let t = (trace + 1.0).sqrt();
        q[3] = 0.5 * t;
        let t = 0.5 / t;
        q[0] = (m[2][1] - m[1][2]) * t;
        q[1] = (m[0][2] - m[2][0]) * t;
        q[2] = (m[1][0] - m[0][1]) * t;
    } else {
        let mut i = 0;
        if m[1][1] > m[0][0] {
            i = 1;
        }
        if m[2][2] > m[i][i] {
            i = 2;
        }
        let j = (i + 1) % 3;
        let k = (j + 1) % 3;
        let t = (m[i][i] - m[j][j] - m[k][k] + 1.0).sqrt();
        q[i] = 0.5 * t;
        let t = 0.5 / t;
        q[3] = (m[k][j] - m[j][k]) * t;
        q[j] = (m[j][i] + m[i][j]) * t;
        q[k] = (m[k][i] + m[i][k]) * t;
    }
    q
}

/// Returns the dot product of two quaternions' coefficients, summed as Eigen's.
fn quaternion_dot(a: &Quat, b: &Quat) -> f64 {
    (a[0] * b[0] + a[2] * b[2]) + (a[1] * b[1] + a[3] * b[3])
}

/// Returns `q` divided by its norm, or `q` unchanged if its norm is zero (Eigen's `normalized()`).
pub(crate) fn quaternion_normalized(q: &Quat) -> Quat {
    let squared_norm = quaternion_dot(q, q);
    if squared_norm > 0.0 {
        let norm = squared_norm.sqrt();
        q.map(|value| value / norm)
    } else {
        *q
    }
}

/// Returns the spherical interpolation from `from` (t = 0) to `to` (t = 1) (Eigen's `slerp`).
pub(crate) fn quaternion_slerp(from: &Quat, t: f64, to: &Quat) -> Quat {
    let one = 1.0 - f64::EPSILON;
    let d = quaternion_dot(from, to);
    let abs_d = d.abs();
    let (scale0, mut scale1) = if abs_d >= one {
        (1.0 - t, t)
    } else {
        let theta = abs_d.acos();
        let sin_theta = theta.sin();
        (((1.0 - t) * theta).sin() / sin_theta, (t * theta).sin() / sin_theta)
    };
    if d < 0.0 {
        scale1 = -scale1;
    }
    std::array::from_fn(|index| scale0 * from[index] + scale1 * to[index])
}

/// Returns the rotation matrix of a unit quaternion (Eigen's `toRotationMatrix`).
pub(crate) fn quaternion_to_matrix(q: &Quat) -> Mat3 {
    let (x, y, z, w) = (q[0], q[1], q[2], q[3]);
    let (tx, ty, tz) = (2.0 * x, 2.0 * y, 2.0 * z);
    let (twx, twy, twz) = (tx * w, ty * w, tz * w);
    let (txx, txy, txz) = (tx * x, ty * x, tz * x);
    let (tyy, tyz, tzz) = (ty * y, tz * y, tz * z);
    [
        [1.0 - (tyy + tzz), txy - twz, txz + twy],
        [txy + twz, 1.0 - (txx + tzz), tyz - twx],
        [txz - twy, tyz + twx, 1.0 - (txx + tyy)],
    ]
}

/// Returns Eigen's `stableNorm()` of a 3-vector: the largest magnitude times the norm of the
/// vector scaled by its inverse.
fn stable_norm3(v: &[f64; 3]) -> f64 {
    let largest = maxi(maxi(v[0].abs(), v[1].abs()), v[2].abs());
    if largest <= 0.0 {
        return 0.0;
    }
    let (scale, inverse_scale) = if 1.0 / largest > f64::MAX {
        (1.0 / f64::MAX, f64::MAX)
    } else {
        (largest, 1.0 / largest)
    };
    scale * squared_norm3(&v.map(|value| value * inverse_scale)).sqrt()
}

/// Returns the angle and unit axis of a rotation matrix (Eigen's `AngleAxisd(matrix)`, through the
/// quaternion).
pub(crate) fn angle_axis(m: &Mat3) -> (f64, [f64; 3]) {
    let q = quaternion_from_matrix(m);
    let vector = [q[0], q[1], q[2]];
    let mut n = norm3(&vector);
    if n < f64::EPSILON {
        n = stable_norm3(&vector);
    }
    if n == 0.0 {
        return (0.0, [1.0, 0.0, 0.0]);
    }
    let angle = 2.0 * n.atan2(q[3].abs());
    if q[3] < 0.0 {
        n = -n;
    }
    (angle, vector.map(|value| value / n))
}
