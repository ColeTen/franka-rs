# Comparison with libfranka C++

`franka-rs` restructures libfranka 0.21.3's API around Rust's types, ownership and error handling,
while sending the robot the same bytes (see [Validation](../introduction.md#validation)).

## Key Differences

| Aspect | libfranka | franka-rs |
|--------|-----------|-----------|
| Exclusive access | Runtime mutexes | `&mut self`, checked at compile time |
| Errors | Exceptions | `FrankaResult` / `FrankaError` |
| Finishing a motion | `motion_finished = true` | `ControlFlow::Break(command)` |
| Callback control | One overloaded `control()` | `control_torques`, `control_joint_positions`, … with a `MotionConfig` |
| Active control | `startTorqueControl()` etc., `readOnce`/`writeOnce` on `ActiveControlBase` | `ActiveTorqueControl`, `ActiveMotionControl<M>` typed by motion |
| Model | URDF + pinocchio | URDF, in Rust (`Model` + `RobotModel` trait) |
| Dependencies | Eigen, Poco, pinocchio, TinyXML2, console_bridge | nalgebra, thiserror, bitflags, socket2, libc, roxmltree |

```rust
// libfranka: robot.control([](const franka::RobotState&, franka::Duration) -> franka::Torques {...});
robot.control_torques(&MotionConfig::default(), |_state, _period| {
    ControlFlow::Continue(Torques::new([0.0; 7]))
})?;

// libfranka: catch (const franka::ControlException& e) { ... e.log ... }
match robot.control_torques(&config, callback) {
    Ok(log) => { /* Vec<LogEntry> */ }
    Err(FrankaError::Control { message, .. }) => eprintln!("{message}"), // log field is empty
    Err(e) => eprintln!("{e}"),
}
```

## API Mapping

| libfranka | franka-rs | Notes |
|-----------|-----------|-------|
| `Robot::control()` | `Robot::control_*()` | Split by command type |
| `Robot::read()`, `readOnce()` | `read()`, `read_once()` | `read` callback returns `bool` (continue) |
| `Robot::start…Control()` | `start_torque_control()`, `start_motion_control::<M>()` | Typed sessions |
| `Robot::loadModel()` | `load_model()` | |
| `Gripper`, `VacuumGripper` | `Gripper`, `VacuumGripper` | [Known header issue](../gripper.md) |
| `RobotState` | `RobotState` | No `m_total`/`I_total`/`F_x_Ctotal` (use `total_load()`); adds `motion_generator_mode` |
| `Torques`, `JointPositions`, … | same names | Newtypes with `Deref` |
| `CartesianPose` | `CartesianPose` | Same column-major matrix; `Option` elbow; `Isometry3` conversions |
| `Duration` | `std::time::Duration` | |
| `Exception` subclasses | `FrankaError` variants | `std::invalid_argument` → `InvalidArgument` |
| `Errors` | `RobotErrors` | `bitflags` |

## Known Behavioral Differences

Found by comparing the sources; none affects the validated control sequences.

| Area | libfranka | franka-rs |
|------|-----------|-----------|
| Real-time scheduling | `kEnforce` (default) sets the highest priority and refuses non-real-time kernels | `RealtimeConfig` stored only |
| TCP replies | Waits indefinitely | 1 s timeout → `Network` |
| Oversized UDP state | Truncated and accepted | Rejected (`Protocol`) |
| UDP receive error | Shuts TCP down first | TCP left open (StopMove can still be sent) |
| "Motion started" while a motion runs | Protocol error | Accepted |
| `load_model` on a mobile robot | Refused before any request | Sends GetRobotModel, then fails |
| Error log | `ControlException::log` filled | `Control::log` empty |
| Misuse (finished session, wrong controller) | `ControlException` / `std::invalid_argument` | `InvalidOperation` ("start multiple motions": `Control` in both) |
