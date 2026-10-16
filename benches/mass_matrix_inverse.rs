//! Benchmark of [`RobotModel::mass_matrix_inverse`] on the FR3 model.

use std::hint::black_box;

use criterion::{criterion_group, criterion_main, Criterion};
use franka_rs::model::{JointVector, Model, RigidBodyInertia, RobotModel};
use nalgebra::{Matrix3, Vector3};

/// Times the mass matrix inverse at a configuration near the default pose with the Franka Hand attached.
fn mass_matrix_inverse(criterion: &mut Criterion) {
    let model = Model::from_urdf(include_str!("../tests/fixtures/fr3_robot.urdf")).unwrap();
    let q = JointVector::from([0.001, 0.001, 0.001, -2.3552, 0.001, 2.3572, 0.001]);
    let payload = RigidBodyInertia::new(
        0.73,
        Vector3::new(0.01, 0.0, 0.03),
        Matrix3::from_diagonal(&Vector3::new(0.001, 0.0025, 0.0017)),
    );
    criterion.bench_function("mass_matrix_inverse", |bencher| {
        bencher.iter(|| model.mass_matrix_inverse(black_box(&q), black_box(&payload)))
    });
}

criterion_group!(benches, mass_matrix_inverse);
criterion_main!(benches);
