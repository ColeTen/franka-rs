# Types & Constants

`types` holds the command and mode types; `constants` the protocol parameters.

## Command Types

| Type | Contents | Used with |
|------|----------|-----------|
| `JointPositions` | `[f64; 7]`, rad | `control_joint_positions` |
| `JointVelocities` | `[f64; 7]`, rad/s | `control_joint_velocities` |
| `Torques` | `[f64; 7]`, Nm, without gravity and friction | `control_torques` |
| `CartesianPose` | `o_t_ee: [f64; 16]` (column-major 4×4), `elbow: Option<[f64; 2]>` | `control_cartesian_pose` |
| `CartesianVelocities` | `linear`, `angular: Vector3<f64>` (m/s, rad/s), `elbow: Option<[f64; 2]>` | `control_cartesian_velocities` |

The joint types are newtypes (`.0`) with `new`, `From<[f64; 7]>` and `Deref`/`DerefMut` to the
array:

```rust
let q = JointPositions::new([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]);
assert_eq!(q[1], -0.785);
let tau: Torques = [0.0; 7].into();
```

### `CartesianPose`

Stores the pose exactly as given, like libfranka's `CartesianPose`; it is checked to be a
homogeneous transformation only when it becomes a command. `to_isometry` returns
`InvalidArgument` for an invalid matrix rather than repairing it.

```rust
let pose = CartesianPose::from_column_major(&state.o_t_ee_c);
let pose = CartesianPose::from_isometry(Isometry3::translation(0.5, 0.0, 0.3))
    .with_elbow([state.elbow[0], state.elbow[1]]);   // [joint-3 angle, joint-4 sign ±1]
let matrix: [f64; 16] = pose.to_column_major();      // exactly as stored
let iso = pose.to_isometry()?;                       // for pose arithmetic
```

### `CartesianVelocities`

```rust
let v = CartesianVelocities::new(Vector3::new(0.1, 0.0, 0.0), Vector3::zeros());
let v = CartesianVelocities::from_array(&[0.1, 0.0, 0.0, 0.0, 0.0, 0.0]) // [vx, vy, vz, wx, wy, wz]
    .with_elbow([state.elbow[0], state.elbow[1]]);
let array: [f64; 6] = v.to_array();
```

## Enumerations

| Enum | Variants |
|------|----------|
| `Frame` | `Joint1`…`Joint7`, `Flange`, `EndEffector` (flange × F_T_EE), `Stiffness` (EE × EE_T_K) |
| `RobotMode` | `Other`, `Idle`, `Move`, `Guiding`, `Reflex`, `UserStopped`, `AutomaticErrorRecovery` |
| `ControllerMode` | `JointImpedance` (the `MotionConfig` default), `CartesianImpedance`, `ExternalController` (torques; not accepted by motion-only control) |
| `MotionGeneratorMode` | `Idle`, `JointPosition`, `JointVelocity`, `CartesianPosition`, `CartesianVelocity`, `None` |
| `RealtimeConfig` | `Enforce`, `Ignore` — stored and returned by `Robot::realtime_config()`, not acted on |

## Constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `DELTA_T` | `1e-3` | Control cycle (s) |
| `NUM_JOINTS` | `7` | Joints |
| `ROBOT_COMMAND_PORT` / `GRIPPER_COMMAND_PORT` / `VACUUM_GRIPPER_COMMAND_PORT` | `1337` / `1338` / `1339` | TCP ports |
| `ROBOT_PROTOCOL_VERSION` / `GRIPPER_PROTOCOL_VERSION` / `VACUUM_GRIPPER_PROTOCOL_VERSION` | `10` / `3` / `1` | Handshake versions |
| `DEFAULT_TIMEOUT_MS` | `1000` | TCP and UDP timeout (ms) |
| `KEEPALIVE_IDLE_SECS` / `KEEPALIVE_INTERVAL_SECS` | `1` / `3` | TCP keepalive (s) |
| `KEEPALIVE_PROBE_COUNT` | `1` | Defined but not used |
