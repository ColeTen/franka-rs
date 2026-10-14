# Motion Control

| Method | Command |
|--------|---------|
| `control_joint_positions` | `JointPositions` |
| `control_joint_velocities` | `JointVelocities` |
| `control_cartesian_pose` | `CartesianPose` |
| `control_cartesian_velocities` | `CartesianVelocities` |
| `control_motion_with_torques` | a motion type plus `Torques` |

Each takes a `MotionConfig` and a callback run every millisecond. The non-callback equivalent is
`Robot::start_motion_control::<M>` ([Active Control](./modules/active-control.md)).

```rust
let config = MotionConfig::default()                          // JointImpedance, limiting on, 100 Hz
    .with_controller_mode(ControllerMode::CartesianImpedance)
    .with_rate_limiting(true)
    .with_cutoff_frequency(100.0);                           // ≥ 1000 Hz: filter off
```

Motion-only control needs an internal controller; `ExternalController` returns
`InvalidOperation`.

## The Callback

It receives the state and the time since its previous call (zero on the first call, then
normally 1 ms — sum it for elapsed time), and returns `ControlFlow::Continue(command)` or
`ControlFlow::Break(command)` to finish.
Start from the first state's commanded values (`q_d`, `o_t_ee_c`): the robot expects the first
command to continue from them.

```rust
// Move joint 4 by 0.5 rad over 3 s.
let mut time = 0.0;
let mut start: Option<[f64; 7]> = None;
robot.control_joint_positions(&MotionConfig::default(), |state, period| {
    time += period.as_secs_f64();
    let mut q = *start.get_or_insert(state.q_d);
    let s = 0.5 * (1.0 - (std::f64::consts::PI * (time / 3.0).min(1.0)).cos());
    q[3] += 0.5 * s;
    if time >= 3.0 { ControlFlow::Break(JointPositions::new(q)) } else { ControlFlow::Continue(JointPositions::new(q)) }
})?;
```

```rust
// Circle of 5 cm radius in x–y over 4 s.
let mut time = 0.0;
let mut start: Option<[f64; 16]> = None;
robot.control_cartesian_pose(&MotionConfig::default(), |state, period| {
    time += period.as_secs_f64();
    let mut pose = *start.get_or_insert(state.o_t_ee_c);
    let angle = 2.0 * std::f64::consts::PI * (time / 4.0).min(1.0);
    pose[12] += 0.05 * angle.cos() - 0.05; // x (column-major index 12)
    pose[13] += 0.05 * angle.sin();        // y
    let command = CartesianPose::from_column_major(&pose);
    if time >= 4.0 { ControlFlow::Break(command) } else { ControlFlow::Continue(command) }
})?;
```

Each cycle the command is low-pass filtered, rate limited and checked, then sent. On `Break`
the final command is resent until the robot stops the motion. Any error — an invalid command, a
robot reflex, a lost connection — or a panic in the callback cancels the motion with StopMove.

## Motion + Torques

Under the external controller with two callbacks: the torque callback runs first and, while it has
not finished, the motion callback; either returning `Break` finishes.

```rust
robot.control_motion_with_torques(
    &MotionConfig::default(),
    |state, _period| ControlFlow::Continue(JointPositions::new(state.q_d)),
    |state, _period| ControlFlow::Continue(Torques::new(compute_impedance_torques(state))),
)?;
```

## Filtering and Rate Limiting

- **Filter** (cutoff < 1000 Hz): first-order, gain `Δt / (Δt + 1/(2π f_c))`; rotations by
  quaternion slerp. See [Low-Pass Filter](./modules/lowpass-filter.md).
- **Rate limiter** (`limit_rate`): libfranka's limits, each reduced by 1e-3 —

| Quantity | Velocity | Acceleration | Jerk |
|----------|----------|--------------|------|
| Joints | from the URDF, position-dependent | 10 rad/s² | 5000 rad/s³ |
| Cartesian translation | 3.0 m/s | 9 m/s² | 4500 m/s³ |
| Cartesian rotation (pose: × 0.99) | 2.5 rad/s | 17 rad/s² | 8500 rad/s³ |
| Elbow | 1.5 rad/s | 10 rad/s² | 5000 rad/s³ |

A command beyond a limit is limited (tracking error); a non-finite value, a non-homogeneous pose
or an elbow sign other than ±1 is rejected with `InvalidArgument`. Active control does not
filter or limit but applies the same checks.
