# Robot API

`Robot` owns the connection and provides state reading, configuration, and control.

| Group | Methods |
|-------|---------|
| Connection | `connect(address)`, `connect_with_config(address, RealtimeConfig)`, `server_version()`, `realtime_config()` |
| State | `read_once()`, `read(callback)` |
| Callback control | `control_joint_positions`, `control_joint_velocities`, `control_cartesian_pose`, `control_cartesian_velocities`, `control_torques`, `control_motion_with_torques` |
| Active control | `start_torque_control()`, `start_motion_control::<M>(ControllerMode)` |
| Configuration | `set_collision_behavior`, `set_joint_impedance`, `set_cartesian_impedance`, `set_guiding_mode`, `set_k_frame`, `set_ee_frame`, `set_load`, `automatic_error_recovery`, `stop` |
| Model | `get_robot_model()` (URDF text), `load_model()` (`Model`) |

All fallible methods return `FrankaResult`.

## Connection

```rust
let robot = Robot::connect("172.16.0.2")?;  // RealtimeConfig::Ignore
let robot = Robot::connect_with_config("172.16.0.2", RealtimeConfig::Enforce)?; // stored only
```

`connect` takes an IP address (not a hostname), performs the handshake, waits for the first state,
and downloads the URDF to read the joint velocity limits used by joint rate limiting (for a mobile
robot's URDF, whose robot name starts with `tmr`, the limits stay all zero). See
[Connecting](../connecting.md).

## State Reading

- `read_once()` returns the newest received state if it is newer than the last one read;
  otherwise it waits for the next.
- `read(callback)` calls `callback(&state)` for each new state until it returns `false`.

## Callback Control

```rust
let config = MotionConfig::default()                       // JointImpedance, limiting on, 100 Hz
    .with_controller_mode(ControllerMode::CartesianImpedance)
    .with_rate_limiting(false)
    .with_cutoff_frequency(50.0);                          // ≥ 1000 Hz: filter off
```

| Method | Command | Controller |
|--------|---------|------------|
| `control_joint_positions` | `JointPositions` | `config.controller_mode` (internal) |
| `control_joint_velocities` | `JointVelocities` | internal |
| `control_cartesian_pose` | `CartesianPose` | internal |
| `control_cartesian_velocities` | `CartesianVelocities` | internal |
| `control_torques` | `Torques` | external |
| `control_motion_with_torques` | `M` + `Torques` (two callbacks) | external |

- Callbacks are `FnMut(&RobotState, Duration) -> ControlFlow<T, T>`; the `Duration` is the time
  since the previous call (zero on the first call, then normally 1 ms). `Continue(cmd)` keeps
  going, `Break(cmd)` finishes.
- Success returns the motion's `Vec<LogEntry>`; on an error the motion is cancelled (StopMove).
- Motion-only methods return `InvalidOperation` for `ExternalController`.
- `&mut self` keeps `robot` unusable inside the callback and while an active session exists.

## Configuration

```rust
// Contact (lower) and collision (upper) thresholds: joint torques (Nm), Cartesian forces (N, Nm)
robot.set_collision_behavior(&CollisionConfig::symmetric([20.0; 7], [40.0; 7], [20.0; 6], [40.0; 6]))?;

robot.set_joint_impedance([600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0])?; // Nm/rad, 0–14250
robot.set_cartesian_impedance([3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0])?; // 10–3000 N/m, 1–300 Nm/rad

// Load: mass (kg), center of mass in the flange frame (m), inertia (kg·m², column-major)
robot.set_load(&LoadConfig::new(0.5, [0.0, 0.0, 0.05], [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]))?;

robot.set_guiding_mode([false, false, true, false, false, true], false)?; // free z and yaw; elbow locked
robot.set_k_frame(ee_t_k)?;    // EE_T_K: stiffness frame in the end-effector frame, column-major [f64; 16]
robot.set_ee_frame(ne_t_ee)?;  // NE_T_EE: end effector in the nominal end-effector frame, column-major [f64; 16]
robot.automatic_error_recovery()?;
robot.stop()?;                 // stop all running motions
```

`CollisionConfig::symmetric` uses the same thresholds for the acceleration and nominal phases;
the struct's eight fields set them separately. The impedance ranges are libfranka's documented
ones.

Motions are started with libfranka's defaults: maximum path and goal deviations of 10.0
(translation), 3.12 (rotation) and 2π (elbow) (`DEFAULT_DEVIATION_*`), no asynchronous motion
generator. `network::RobotCommand` is the low-level helper that sends these TCP commands.
