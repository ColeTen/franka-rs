# Active Control

Active control replaces the callback with a session handle you drive from your own loop (for
external frameworks, state machines, async code). It mirrors libfranka's `startTorqueControl` /
`startJointPositionControl` … with `readOnce`/`writeOnce`.

| Session | Created by | Methods |
|---------|------------|---------|
| `ActiveTorqueControl` | `robot.start_torque_control()` | `read_state`, `write_torques(&Torques)`, `finish(&Torques)` |
| `ActiveMotionControl<M>` | `robot.start_motion_control::<M>(controller_mode)` | `read_state`, `write_motion(&M)`, `write_motion_with_torques(&M, &Torques)`, `finish(&M)`, `finish_with_torques(&M, &Torques)` |

`M` is `JointPositions`, `JointVelocities`, `CartesianPose` or `CartesianVelocities`.

## Torque Session

```rust
let mut ctrl = robot.start_torque_control()?;
let mut state = ctrl.read_state()?;
loop {
    let tau = Torques::new(compute_torques(&state));
    if should_stop(&state) {
        ctrl.finish(&tau)?;   // resend with the finished flag until stopped, then await the Move response
        break;
    }
    state = ctrl.write_torques(&tau)?; // sends, then returns the next state
}
```

Use the state `write_torques` returns; calling `read_state` as well would skip a state each cycle.

## Motion Session

```rust
let mut ctrl = robot.start_motion_control::<JointPositions>(ControllerMode::JointImpedance)?;
let state = ctrl.read_state()?;
ctrl.write_motion(&JointPositions::new(state.q_d))?;
ctrl.finish(&JointPositions::new(state.q_d))?;
```

With `ControllerMode::ExternalController` every command carries torques
(`write_motion_with_torques`, `finish_with_torques`); otherwise torques are rejected.

## Behavior

- **States, not periods:** `read_state` and `write_*` return the `RobotState` only (libfranka's
  `readOnce` also returns the period); use `state.time` differences for the period.
- **Torque `finish`** returns `InvalidOperation` if a motion generator is still running, as
  libfranka does.
- **No filtering or rate limiting**, as in libfranka's active control. Commands are checked
  (finite values, homogeneous pose, elbow sign ±1); an invalid one returns `InvalidArgument` and
  nothing is sent.
- **Errors**: `read_state` (and `write_*`, which read the next state) fail once the robot has left
  the motion (user stop, reflex); `finish` fails if the robot answers with an abort status; every
  method returns `InvalidOperation` after the session has finished.
- **Drop**: a session dropped without a successful `finish` (error, `?`, panic) is cancelled with
  StopMove, as libfranka's destructor does; after `finish` nothing is sent.
- **Borrowing**: the session holds `&mut` of the robot's connection, so the robot cannot be used
  and no second session can start until it is dropped — checked at compile time.
