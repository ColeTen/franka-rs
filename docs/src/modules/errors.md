# Error Handling

- **`FrankaError`** — errors of franka-rs operations; `FrankaResult<T> = Result<T, FrankaError>`
  is returned by every fallible public method.
- **`RobotErrors`** — the robot's 41 safety and error flags (`bitflags`), in
  `RobotState::current_errors` and `last_motion_errors`.

## `FrankaError`

| Variant | Cause | Fields |
|---------|-------|--------|
| `Network` | TCP/UDP failure, timeout, closed connection | `message`, `source: Option<io::Error>` |
| `Protocol` | Malformed or unexpected message | `message` |
| `IncompatibleVersion` | Handshake version mismatch | `server_version`, `library_version` |
| `Control` | The robot ended or rejected a running motion (the message includes `last_motion_errors`), or misuse libfranka reports as a `ControlException` | `message`, `log` (always empty) |
| `Command` | The robot rejected a command | `message` |
| `Model` | Unusable URDF: invalid XML, not a serial chain of seven revolute joints ending in `link8`, missing joint velocity limits | `message` |
| `InvalidOperation` | Operation in the wrong state (finished session, wrong controller mode) | `message` |
| `InvalidArgument` | Invalid command or filter/limiter input: non-finite value, cutoff or sample time out of range, non-homogeneous pose, elbow sign not ±1 (libfranka's `std::invalid_argument`) | `message` |

```rust
match robot.control_torques(&config, callback) {
    Ok(log) => println!("{} cycles", log.len()),
    Err(FrankaError::Control { message, .. }) => eprintln!("Motion ended: {message}"),
    Err(FrankaError::InvalidArgument { message }) => eprintln!("Invalid command: {message}"),
    Err(FrankaError::Network { message, .. }) => eprintln!("Network: {message}"),
    Err(e) => eprintln!("{e}"),
}
```

Constructors: `FrankaError::network(message)`, `FrankaError::network_with_source(message, io_error)`.

## `RobotErrors`

| Category | Flags |
|----------|-------|
| Limits | `JOINT_POSITION_LIMITS_VIOLATION`, `CARTESIAN_POSITION_LIMITS_VIOLATION`, `JOINT_VELOCITY_VIOLATION`, `CARTESIAN_VELOCITY_VIOLATION`, `TAU_J_RANGE_VIOLATION`, `POWER_LIMIT_VIOLATION` |
| Collision | `SELF_COLLISION_AVOIDANCE_VIOLATION`, `JOINT_REFLEX`, `CARTESIAN_REFLEX` |
| Motion generator | `JOINT_MOTION_GENERATOR_*` and `CARTESIAN_MOTION_GENERATOR_*` (limits, discontinuities, elbow), `JOINT_POSITION_MOTION_GENERATOR_START_POSE_INVALID`, `CARTESIAN_POSITION_MOTION_GENERATOR_START_POSE_INVALID`, `CARTESIAN_POSITION_MOTION_GENERATOR_INVALID_FRAME`, `START_ELBOW_SIGN_INCONSISTENT`, `MAX_GOAL_POSE_DEVIATION_VIOLATION`, `MAX_PATH_POSE_DEVIATION_VIOLATION`, `CARTESIAN_SPLINE_MOTION_GENERATOR_VIOLATION`, `JOINT_VIA_MOTION_GENERATOR_PLANNING_JOINT_LIMIT_VIOLATION`, `JOINT_P2P_INSUFFICIENT_TORQUE_FOR_PLANNING` |
| Controller | `CONTROLLER_TORQUE_DISCONTINUITY`, `FORCE_CONTROL_SAFETY_VIOLATION`, `FORCE_CONTROLLER_DESIRED_FORCE_TOLERANCE_VIOLATION`, `INSTABILITY_DETECTED`, `JOINT_MOVE_IN_WRONG_DIRECTION` |
| Other | `COMMUNICATION_CONSTRAINTS_VIOLATION`, `CARTESIAN_VELOCITY_PROFILE_SAFETY_VIOLATION`, `BASE_ACCELERATION_INITIALIZATION_TIMEOUT`, `BASE_ACCELERATION_INVALID_READING` |

```rust
let state = robot.read_once()?;
if state.last_motion_errors.contains(RobotErrors::CARTESIAN_REFLEX) {
    println!("Last motion was stopped by a Cartesian reflex");
}
println!("Active errors: {:?}", state.current_errors);
```

The robot sends the flags as `[u8; 41]`, converted to `[bool; 41]`; `RobotErrors::from_bool_array`
builds the set from that, and `has_errors` reports whether any is active.

## Recovery

After a reflex, `robot.automatic_error_recovery()` clears the errors; if the robot rejects it,
the error is `FrankaError::Command`. See [Safety & Error Recovery](../design/safety.md).
