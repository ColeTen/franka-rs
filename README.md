# franka-rs

**Idiomatic Rust interface for the Franka Research 3 robot.**

`franka-rs` is a pure-Rust implementation of the [Franka Control Interface (FCI)](https://frankaemika.github.io/docs/), providing a safe, performant, and ergonomic API for controlling Franka Emika Panda and FR3 robots at 1 kHz.

> [!NOTE]
> This project is under active development and is not yet published on crates.io.

## Why franka-rs?

The official C++ library ([libfranka](https://github.com/frankaemika/libfranka)) relies on CMake, Eigen, Poco, and a dynamically loaded model library (`.so`). Cross-compilation is painful, runtime errors are common, and nothing prevents concurrent access to the robot at compile time.

`franka-rs` redesigns the interface from scratch to leverage Rust's type system, ownership model, and error handling:

| | libfranka (C++) | franka-rs |
|---|---|---|
| **Build** | CMake + C++ compiler + Eigen + Poco | `cargo build` |
| **Model library** | Dynamic `.so` loaded at runtime | Pure Rust (no FFI) |
| **Concurrent access** | Runtime mutexes | Compile-time borrow checker |
| **Error handling** | C++ exceptions | `Result<T, FrankaError>` |
| **Motion completion** | `finished` bool field | `ControlFlow::Break` / `Continue` |
| **Cross-compilation** | Complex (need target `.so`) | Standard Rust cross-compile |

## Features

- **Pure Rust** -- no C/C++ dependencies, no FFI, no dynamic library loading
- **Compile-time safety** -- the borrow checker enforces single-writer access to the robot; the type system prevents sending the wrong command type
- **Real-time capable** -- synchronous 1 kHz control loop with automatic rate limiting and low-pass filtering
- **Full kinematics & dynamics** -- forward kinematics, Jacobians (body and spatial), mass matrix, Coriolis vector, and gravity compensation, all computed in pure Rust using modified DH parameters
- **Multiple control modes** -- joint position, joint velocity, Cartesian pose, Cartesian velocity, direct torque, and combined motion + torque
- **Active (non-callback) control** -- imperative read/write interface with RAII cleanup, for integration with external control loops
- **Gripper support** -- parallel gripper (grasp, move, homing) and vacuum gripper (vacuum profiles P0--P3, drop-off)
- **Structured error recovery** -- `FrankaError` enum with log ring buffer for post-mortem diagnostics; `automatic_error_recovery()` to clear faults
- **Configurable safety** -- collision/contact thresholds, joint/Cartesian impedance, load parameters, guiding mode

## Quick Start

Add `franka-rs` to your `Cargo.toml`:

```toml
[dependencies]
franka-rs = { git = "https://github.com/cten-ucla/franka-rs" }
```

Connect and read robot state:

```rust
use franka_rs::robot::Robot;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut robot = Robot::connect("172.16.0.2")?;
    let state = robot.read_once()?;
    println!("Joint positions: {:?}", state.q);
    println!("EE pose: {:?}", &state.o_t_ee[12..15]); // xyz translation
    Ok(())
}
```

## Examples

### Gravity Compensation (Zero-Torque Control)

```rust
use franka_rs::robot::Robot;
use franka_rs::model::Model;
use franka_rs::types::Torques;
use std::ops::ControlFlow;

let mut robot = Robot::connect("172.16.0.2")?;
let model = Model::new();

robot.control_torques(|state, duration| {
    let gravity = model.gravity_from_state(state);

    if duration.as_secs_f64() >= 10.0 {
        ControlFlow::Break(Torques::new(gravity))
    } else {
        ControlFlow::Continue(Torques::new(gravity))
    }
})?;
```

### Joint Position Trajectory

```rust
use franka_rs::types::JointPositions;
use std::ops::ControlFlow;

let q_start = robot.read_once()?.q;

robot.control_joint_positions(|_state, duration| {
    let t = duration.as_secs_f64();
    if t >= 3.0 {
        let mut q = q_start;
        q[3] += 0.5;
        return ControlFlow::Break(JointPositions::new(q));
    }
    // Cosine interpolation for smooth acceleration
    let s = 0.5 * (1.0 - (std::f64::consts::PI * t / 3.0).cos());
    let mut q = q_start;
    q[3] += 0.5 * s;
    ControlFlow::Continue(JointPositions::new(q))
})?;
```

### Impedance Control

```rust
use franka_rs::types::Torques;
use std::ops::ControlFlow;

let model = Model::new();
let stiffness = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0];
let damping = [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0];
let q_desired = robot.read_once()?.q;

robot.control_torques(|state, duration| {
    let gravity = model.gravity_from_state(state);
    let mut tau = [0.0; 7];
    for i in 0..7 {
        tau[i] = gravity[i]
            + stiffness[i] * (q_desired[i] - state.q[i])
            - damping[i] * state.dq[i];
    }
    if duration.as_secs_f64() >= 30.0 {
        ControlFlow::Break(Torques::new(tau))
    } else {
        ControlFlow::Continue(Torques::new(tau))
    }
})?;
```

### Active (Non-Callback) Control

```rust
let mut ctrl = robot.start_torque_control()?;

loop {
    let state = ctrl.read_state()?;
    let tau = compute_my_torques(&state);
    if done() {
        ctrl.write_torques_finish(Torques::new(tau))?;
        break;
    }
    ctrl.write_torques(Torques::new(tau))?;
}
// ctrl dropped here -- RAII sends stop command automatically
```

### Gripper

```rust
use franka_rs::gripper::Gripper;

let mut gripper = Gripper::connect("172.16.0.2")?;
gripper.homing()?;
gripper.grasp(0.04, 0.1, 60.0, 0.005, 0.005)?;
gripper.move_fingers(0.08, 0.1)?;
```

## Control Modes

| Method | Command Type | Use Case |
|---|---|---|
| `control_torques` | `Torques` | Direct torque control, impedance control, force control |
| `control_joint_positions` | `JointPositions` | Joint-space trajectory tracking |
| `control_joint_velocities` | `JointVelocities` | Velocity-resolved control |
| `control_cartesian_pose` | `CartesianPose` | Task-space pose tracking |
| `control_cartesian_velocities` | `CartesianVelocities` | Task-space velocity control |
| `control_joint_positions_with_torques` | `(JointPositions, Torques)` | Combined motion + torque overlay |

All control callbacks receive the current `RobotState` and elapsed `Duration`, and return `ControlFlow::Continue(cmd)` to keep running or `ControlFlow::Break(cmd)` to stop.

## Model API

Pure Rust kinematics and dynamics -- no external `.so` required:

| Function | Output | Description |
|---|---|---|
| `pose(frame, q)` | `[f64; 16]` | Forward kinematics (4x4 homogeneous, column-major) |
| `zero_jacobian(frame, q)` | `[f64; 42]` | 6x7 spatial Jacobian in base frame |
| `body_jacobian(frame, q)` | `[f64; 42]` | 6x7 body Jacobian in target frame |
| `mass(q, ...)` | `[f64; 49]` | 7x7 joint-space inertia matrix |
| `coriolis(q, dq, ...)` | `[f64; 7]` | Coriolis and centrifugal force vector |
| `gravity(q, ...)` | `[f64; 7]` | Gravity compensation torques |

Every function has a `_from_state` variant that extracts parameters directly from `RobotState`.

## Architecture

```
franka-rs
├── robot          # Main public interface (connect, configure, control)
├── active_control # Non-callback streaming control (ActiveTorqueControl, ActiveMotionControl)
├── model          # Pure Rust kinematics & dynamics
├── gripper        # Parallel gripper interface
├── vacuum_gripper # Vacuum gripper interface
├── control_loop   # 1 kHz real-time loop orchestration
├── rate_limiting  # Joint/Cartesian rate and jerk limiting
├── lowpass_filter # Butterworth filter with quaternion SLERP for rotations
├── logging        # Ring buffer of state/command pairs for diagnostics
├── network        # TCP + UDP socket management and framing
├── wire           # Binary packed structs matching the FCI protocol
├── types          # Domain newtypes (JointPositions, Torques, CartesianPose, ...)
├── errors         # Error hierarchy (FrankaError, RobotErrors via bitflags)
└── constants      # DH parameters, joint limits, protocol constants
```

## Safety

`franka-rs` implements multiple layers of safety:

1. **Compile-time** -- borrow checker prevents concurrent robot access; type system prevents sending wrong command types; `ControlFlow` enum makes stop signaling explicit
2. **Library runtime** -- automatic rate limiting (jerk, acceleration, velocity, torque rate), low-pass filtering, NaN/Inf rejection, RAII cleanup on drop
3. **Robot controller** -- collision detection, joint limits, self-collision avoidance, reflex response
4. **Hardware** -- emergency stop button, joint brakes on power loss

## Supported Hardware

| Robot | Status |
|---|---|
| Franka Emika Panda | Supported |
| Franka Research 3 (FR3) | Supported |

Requires **Franka Control Interface (FCI)** firmware. The robot must be in FCI mode (not Desk mode).

## Dependencies

| Crate | Purpose |
|---|---|
| [nalgebra](https://crates.io/crates/nalgebra) | Linear algebra (matrices, quaternions, isometries) |
| [thiserror](https://crates.io/crates/thiserror) | Derive macros for error types |
| [bitflags](https://crates.io/crates/bitflags) | Robot error flag representation |
| [socket2](https://crates.io/crates/socket2) | Low-level TCP/UDP socket control |
