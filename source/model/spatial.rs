//! Spatial algebra for rigid-body kinematics and dynamics.
//!
//! Spatial vectors are stored as separate linear and angular 3-vectors, and rigid
//! transforms are `nalgebra::Isometry3` values mapping child coordinates to parent
//! coordinates.

use nalgebra::{Isometry3, Matrix3, Matrix6, Vector3, Vector6};

/// Mass properties of a rigid body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyInertia {
    /// Mass in kg.
    pub mass: f64,
    /// Center of mass, expressed in the body's reference frame, in m.
    pub center_of_mass: Vector3<f64>,
    /// Rotational inertia about the center of mass, in the body's reference frame axes, in kg·m².
    pub rotational_inertia: Matrix3<f64>,
}

/// Spatial velocity or acceleration (twist) of a body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SpatialMotion {
    /// Linear component, in m/s or m/s².
    pub linear: Vector3<f64>,
    /// Angular component, in rad/s or rad/s².
    pub angular: Vector3<f64>,
}

/// Spatial force (wrench) acting on a body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SpatialForce {
    /// Force component, in N.
    pub linear: Vector3<f64>,
    /// Torque component, in N·m.
    pub angular: Vector3<f64>,
}

impl RigidBodyInertia {
    /// Creates an inertia from mass, center of mass, and rotational inertia about the center of mass.
    pub fn new(mass: f64, center_of_mass: Vector3<f64>, rotational_inertia: Matrix3<f64>) -> Self {
        Self {
            mass,
            center_of_mass,
            rotational_inertia,
        }
    }

    /// Returns the inertia of a massless body.
    pub fn zero() -> Self {
        Self::new(0.0, Vector3::zeros(), Matrix3::zeros())
    }

    /// Re-expresses this inertia in a parent frame, given the pose of this body's frame in that parent.
    pub fn transformed(&self, parent_from_body: &Isometry3<f64>) -> Self {
        let rotation = parent_from_body.rotation.to_rotation_matrix();
        Self::new(
            self.mass,
            parent_from_body.translation.vector + rotation * self.center_of_mass,
            rotation * self.rotational_inertia * rotation.transpose(),
        )
    }

    /// Returns the inertia of the rigid union of two bodies expressed in the same frame.
    ///
    /// If both bodies are massless, the combined center of mass is the origin.
    pub fn combine(&self, other: &Self) -> Self {
        let combined_mass = self.mass + other.mass;
        let inverse_combined_mass = 1.0 / combined_mass.max(f64::EPSILON);
        let center_offset = self.center_of_mass - other.center_of_mass;
        let center_offset_skew = center_offset.cross_matrix();
        Self::new(
            combined_mass,
            (self.mass * self.center_of_mass + other.mass * other.center_of_mass)
                * inverse_combined_mass,
            self.rotational_inertia + other.rotational_inertia
                - (self.mass * other.mass * inverse_combined_mass)
                    * (center_offset_skew * center_offset_skew),
        )
    }

    /// Returns this inertia as a 6×6 matrix mapping a motion `[linear; angular]` to its momentum
    /// `[linear; angular]` (pinocchio's `Inertia::matrix`).
    pub(crate) fn spatial_matrix(&self) -> Matrix6<f64> {
        let lever = self.center_of_mass.cross_matrix();
        let mut matrix = Matrix6::zeros();
        matrix.fixed_view_mut::<3, 3>(0, 0).copy_from(&(self.mass * Matrix3::identity()));
        matrix.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-self.mass * lever));
        matrix.fixed_view_mut::<3, 3>(3, 0).copy_from(&(self.mass * lever));
        matrix
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(self.rotational_inertia - self.mass * lever * lever));
        matrix
    }

    /// Returns pinocchio's `Inertia::variation(motion)`: the 6×6 matrix `motion×* I − I motion×`,
    /// the time derivative of this inertia when its frame moves with `motion`.
    pub(crate) fn variation(&self, motion: &SpatialMotion) -> Matrix6<f64> {
        let lever = self.center_of_mass;
        let scaled_linear = motion.linear * self.mass;
        let scaled_angular = motion.angular * self.mass;
        let linear_angular = -scaled_linear.cross_matrix() - skew_square(&scaled_angular, &lever)
            + skew_square(&lever, &scaled_angular);
        let about_origin = self.rotational_inertia - self.mass * lever.cross_matrix() * lever.cross_matrix();
        let angular_angular = -skew_square(&scaled_linear, &lever) - skew_square(&lever, &scaled_linear)
            - about_origin * motion.angular.cross_matrix()
            + motion.angular.cross_matrix() * about_origin;
        let mut matrix = Matrix6::zeros();
        matrix.fixed_view_mut::<3, 3>(0, 3).copy_from(&linear_angular);
        matrix.fixed_view_mut::<3, 3>(3, 0).copy_from(&linear_angular.transpose());
        matrix.fixed_view_mut::<3, 3>(3, 3).copy_from(&angular_angular);
        matrix
    }

    /// Returns the momentum (or inertial force) produced by this inertia moving with `motion`.
    pub(crate) fn apply(&self, motion: &SpatialMotion) -> SpatialForce {
        let linear =
            self.mass * (motion.linear - self.center_of_mass.cross(&motion.angular));
        let angular =
            self.rotational_inertia * motion.angular + self.center_of_mass.cross(&linear);
        SpatialForce { linear, angular }
    }
}

/// Returns the matrix of `w ↦ u × (v × w)` (pinocchio's `skewSquare(u, v)`).
fn skew_square(u: &Vector3<f64>, v: &Vector3<f64>) -> Matrix3<f64> {
    v * u.transpose() - u.dot(v) * Matrix3::identity()
}

impl SpatialMotion {
    /// Returns this motion as the vector `[linear; angular]`.
    pub(crate) fn to_vector(self) -> Vector6<f64> {
        Vector6::new(self.linear.x, self.linear.y, self.linear.z, self.angular.x, self.angular.y, self.angular.z)
    }

    /// Returns the zero motion.
    pub fn zero() -> Self {
        Self {
            linear: Vector3::zeros(),
            angular: Vector3::zeros(),
        }
    }

    /// Returns the motion of a revolute joint about `axis` at `rate`.
    pub fn revolute(axis: &Vector3<f64>, rate: f64) -> Self {
        Self {
            linear: Vector3::zeros(),
            angular: axis * rate,
        }
    }

    /// Re-expresses this motion from a child frame into its parent frame.
    pub fn act(&self, parent_from_child: &Isometry3<f64>) -> Self {
        let angular = parent_from_child.rotation * self.angular;
        let linear = parent_from_child.rotation * self.linear
            + parent_from_child.translation.vector.cross(&angular);
        Self { linear, angular }
    }

    /// Re-expresses this motion from a parent frame into a child frame.
    pub fn act_inverse(&self, parent_from_child: &Isometry3<f64>) -> Self {
        let inverse_rotation = parent_from_child.rotation.inverse();
        Self {
            linear: inverse_rotation
                * (self.linear - parent_from_child.translation.vector.cross(&self.angular)),
            angular: inverse_rotation * self.angular,
        }
    }

    /// Returns the spatial cross product of this motion with another motion.
    pub fn cross_motion(&self, other: &Self) -> Self {
        Self {
            linear: self.angular.cross(&other.linear) + self.linear.cross(&other.angular),
            angular: self.angular.cross(&other.angular),
        }
    }

    /// Returns the spatial cross product of this motion with a force.
    pub fn cross_force(&self, force: &SpatialForce) -> SpatialForce {
        SpatialForce {
            linear: self.angular.cross(&force.linear),
            angular: self.angular.cross(&force.angular) + self.linear.cross(&force.linear),
        }
    }
}

impl std::ops::Add for SpatialMotion {
    type Output = Self;

    /// Sums two motions expressed in the same frame.
    fn add(self, other: Self) -> Self {
        Self {
            linear: self.linear + other.linear,
            angular: self.angular + other.angular,
        }
    }
}

impl SpatialForce {
    /// Returns the matrix pinocchio's `addForceCrossMatrix` adds for this force: `-[linear]×` in the
    /// two off-diagonal blocks and `-[angular]×` in the angular block.
    pub(crate) fn cross_matrix(&self) -> Matrix6<f64> {
        let mut matrix = Matrix6::zeros();
        matrix.fixed_view_mut::<3, 3>(0, 3).copy_from(&(-self.linear).cross_matrix());
        matrix.fixed_view_mut::<3, 3>(3, 0).copy_from(&(-self.linear).cross_matrix());
        matrix.fixed_view_mut::<3, 3>(3, 3).copy_from(&(-self.angular).cross_matrix());
        matrix
    }

    /// Re-expresses this force from a child frame into its parent frame.
    pub fn act(&self, parent_from_child: &Isometry3<f64>) -> Self {
        let linear = parent_from_child.rotation * self.linear;
        let angular = parent_from_child.rotation * self.angular
            + parent_from_child.translation.vector.cross(&linear);
        Self { linear, angular }
    }
}

impl std::ops::Add for SpatialForce {
    type Output = Self;

    /// Sums two forces expressed in the same frame.
    fn add(self, other: Self) -> Self {
        Self {
            linear: self.linear + other.linear,
            angular: self.angular + other.angular,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    #[test]
    fn combine_applies_parallel_axis_theorem() {
        let point_a = RigidBodyInertia::new(1.0, Vector3::new(1.0, 0.0, 0.0), Matrix3::zeros());
        let point_b = RigidBodyInertia::new(1.0, Vector3::new(-1.0, 0.0, 0.0), Matrix3::zeros());
        let combined = point_a.combine(&point_b);
        assert_eq!(combined.mass, 2.0);
        assert!(combined.center_of_mass.norm() < 1e-12);
        let expected = Matrix3::from_diagonal(&Vector3::new(0.0, 2.0, 2.0));
        assert!((combined.rotational_inertia - expected).norm() < 1e-12);
    }

    #[test]
    fn combine_with_zero_is_identity() {
        let body = RigidBodyInertia::new(
            2.5,
            Vector3::new(0.1, -0.2, 0.3),
            Matrix3::from_diagonal(&Vector3::new(0.01, 0.02, 0.03)),
        );
        let combined = body.combine(&RigidBodyInertia::zero());
        assert!((combined.mass - body.mass).abs() < 1e-12);
        assert!((combined.center_of_mass - body.center_of_mass).norm() < 1e-12);
        assert!((combined.rotational_inertia - body.rotational_inertia).norm() < 1e-12);
    }

    #[test]
    fn motion_act_and_act_inverse_round_trip() {
        let transform = Isometry3::from_parts(
            Translation3::new(0.3, -0.1, 0.7),
            UnitQuaternion::from_euler_angles(0.4, -0.2, 1.1),
        );
        let motion = SpatialMotion {
            linear: Vector3::new(1.0, 2.0, 3.0),
            angular: Vector3::new(-0.5, 0.25, 0.75),
        };
        let round_trip = motion.act(&transform).act_inverse(&transform);
        assert!((round_trip.linear - motion.linear).norm() < 1e-12);
        assert!((round_trip.angular - motion.angular).norm() < 1e-12);
    }

    #[test]
    fn transformed_inertia_preserves_kinetic_energy() {
        let transform = Isometry3::from_parts(
            Translation3::new(0.3, -0.1, 0.7),
            UnitQuaternion::from_euler_angles(0.4, -0.2, 1.1),
        );
        let body = RigidBodyInertia::new(
            1.7,
            Vector3::new(0.05, 0.02, -0.04),
            Matrix3::new(0.02, 0.001, 0.0, 0.001, 0.03, 0.002, 0.0, 0.002, 0.01),
        );
        let motion_in_body = SpatialMotion {
            linear: Vector3::new(0.2, -0.4, 0.1),
            angular: Vector3::new(0.3, 0.6, -0.9),
        };
        let energy_in_body = {
            let momentum = body.apply(&motion_in_body);
            momentum.linear.dot(&motion_in_body.linear) + momentum.angular.dot(&motion_in_body.angular)
        };
        let motion_in_parent = motion_in_body.act(&transform);
        let body_in_parent = body.transformed(&transform);
        let energy_in_parent = {
            let momentum = body_in_parent.apply(&motion_in_parent);
            momentum.linear.dot(&motion_in_parent.linear)
                + momentum.angular.dot(&motion_in_parent.angular)
        };
        assert!((energy_in_body - energy_in_parent).abs() < 1e-12);
    }

    /// Returns a body with an off-center mass and a full rotational inertia.
    fn asymmetric_body() -> RigidBodyInertia {
        RigidBodyInertia::new(
            1.3,
            Vector3::new(0.07, -0.03, 0.05),
            Matrix3::new(0.02, 0.001, -0.002, 0.001, 0.03, 0.004, -0.002, 0.004, 0.015),
        )
    }

    /// Returns a motion with every component nonzero.
    fn test_motion() -> SpatialMotion {
        SpatialMotion { linear: Vector3::new(0.4, -0.7, 0.2), angular: Vector3::new(-0.3, 0.9, 0.5) }
    }

    #[test]
    fn spatial_matrix_maps_motion_to_momentum() {
        let body = asymmetric_body();
        let motion = test_motion();
        let momentum = body.apply(&motion);
        let expected = Vector6::new(
            momentum.linear.x, momentum.linear.y, momentum.linear.z,
            momentum.angular.x, momentum.angular.y, momentum.angular.z,
        );
        assert!((body.spatial_matrix() * motion.to_vector() - expected).norm() < 1e-12);
    }

    #[test]
    fn skew_square_is_double_cross_product() {
        let (u, v, w) = (Vector3::new(1.0, -2.0, 0.5), Vector3::new(0.3, 0.8, -1.1), Vector3::new(-0.6, 0.2, 0.9));
        assert!((skew_square(&u, &v) * w - u.cross(&v.cross(&w))).norm() < 1e-12);
    }

    #[test]
    fn variation_is_motion_cross_star_inertia_minus_inertia_motion_cross() {
        // With [linear; angular] ordering: motion× = [[ω×, v×], [0, ω×]] and motion×* = −(motion×)ᵀ.
        let body = asymmetric_body();
        let motion = test_motion();
        let mut motion_cross = Matrix6::zeros();
        motion_cross.fixed_view_mut::<3, 3>(0, 0).copy_from(&motion.angular.cross_matrix());
        motion_cross.fixed_view_mut::<3, 3>(0, 3).copy_from(&motion.linear.cross_matrix());
        motion_cross.fixed_view_mut::<3, 3>(3, 3).copy_from(&motion.angular.cross_matrix());
        let force_cross = -motion_cross.transpose();
        let inertia = body.spatial_matrix();
        let expected = force_cross * inertia - inertia * motion_cross;
        assert!((body.variation(&motion) - expected).norm() < 1e-12);
    }

    #[test]
    fn force_cross_matrix_maps_motion_to_negated_cross_product() {
        // addForceCrossMatrix's matrix X satisfies X·m = m ×* f for force f (pinocchio's convention).
        let force = SpatialForce { linear: Vector3::new(0.5, -1.0, 2.0), angular: Vector3::new(-0.2, 0.4, 0.1) };
        let motion = test_motion();
        let expected = motion.cross_force(&force);
        let expected = Vector6::new(
            expected.linear.x, expected.linear.y, expected.linear.z,
            expected.angular.x, expected.angular.y, expected.angular.z,
        );
        assert!((force.cross_matrix() * motion.to_vector() - expected).norm() < 1e-12);
    }
}
