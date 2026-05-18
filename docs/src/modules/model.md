# Model (Kinematics & Dynamics)

## Overview

The `model` module computes the kinematic and dynamic properties of the Franka robot: forward kinematics, Jacobians, mass matrix, Coriolis forces, and gravity compensation. It uses modified Denavit-Hartenberg parameters and identified link inertial parameters.

```mermaid
classDiagram
    class Model {
        -[LinkParams; 7] link_params
        -Matrix4~f64~ f_t_ee
        -Matrix4~f64~ ee_t_k
        +new() Self
        +with_frames(f_t_ee, ee_t_k) Self
        +set_f_t_ee(&[f64; 16])
        +set_ee_t_k(&[f64; 16])
        +pose(Frame, &[f64;7]) [f64;16]
        +body_jacobian(Frame, &[f64;7]) [f64;42]
        +zero_jacobian(Frame, &[f64;7]) [f64;42]
        +mass(&[f64;7], f64, &[f64;3], &[f64;9]) [f64;49]
        +coriolis(&[f64;7], &[f64;7], f64, &[f64;3], &[f64;9]) [f64;7]
        +gravity(&[f64;7], f64, &[f64;3], &[f64;3]) [f64;7]
    }

    class LinkParams {
        +f64 mass
        +[f64; 3] center_of_mass
        +[f64; 9] inertia
    }

    Model --> LinkParams : contains 7
    Model --> kinematics : uses
    Model --> dynamics : uses

    note for Model "All _from_state() variants\ntake &RobotState directly"
```

## Frame Chain

```mermaid
flowchart LR
    BASE["Base<br/>(World)"] --> J1["Joint 1"] --> J2["Joint 2"] --> J3["Joint 3"] --> J4["Joint 4"] --> J5["Joint 5"] --> J6["Joint 6"] --> J7["Joint 7"] --> FL["Flange"]
    FL -->|"F_T_EE<br/>(configurable)"| EE["End Effector"]
    EE -->|"EE_T_K<br/>(configurable)"| K["Stiffness Frame"]
```

The `Frame` enum selects any frame in this chain for kinematics computation.

## Creating a Model

```rust
use franka_rs::model::Model;

// Default: identity transforms for F_T_EE and EE_T_K
let model = Model::new();

// With custom tool transform
let f_t_ee: [f64; 16] = /* your tool frame */;
let ee_t_k: [f64; 16] = /* your stiffness frame */;
let model = Model::with_frames(&f_t_ee, &ee_t_k);

// Modify after creation
let mut model = Model::new();
model.set_f_t_ee(&f_t_ee);
model.set_ee_t_k(&ee_t_k);
```

## Kinematics

### Forward Kinematics (`pose`)

Computes the 4x4 homogeneous transformation (column-major) of any frame:

```rust
let q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]; // home position
let ee_pose = model.pose(Frame::EndEffector, &q);

// Extract translation
let x = ee_pose[12]; // column-major: translation is indices 12, 13, 14
let y = ee_pose[13];
let z = ee_pose[14];
```

From robot state:
```rust
let state = robot.read_once()?;
let ee_pose = model.pose_from_state(Frame::EndEffector, &state);
```

### Body Jacobian (`body_jacobian`)

6x7 matrix (column-major) mapping joint velocities to a twist expressed **in the target frame**:

```rust
let j_body = model.body_jacobian(Frame::EndEffector, &q);
// j_body is [f64; 42] (6 rows × 7 cols, column-major)
```

### Zero (Spatial) Jacobian (`zero_jacobian`)

6x7 matrix mapping joint velocities to a twist expressed **in the base frame**:

```rust
let j_zero = model.zero_jacobian(Frame::EndEffector, &q);
// Use for task-space control: F_task = J^T * tau
```

## Dynamics

### Mass Matrix (`mass`)

7x7 joint-space inertia matrix (column-major):

```rust
let m = model.mass(&q, load_mass, &load_com, &load_inertia);
// m is [f64; 49] (7×7, column-major)
// M(q) · ddq + C(q, dq) + g(q) = tau
```

### Coriolis Vector (`coriolis`)

7-element Coriolis and centrifugal force vector:

```rust
let c = model.coriolis(&q, &dq, load_mass, &load_com, &load_inertia);
// c is [f64; 7]
```

### Gravity Vector (`gravity`)

7-element gravity compensation torque vector:

```rust
let g = model.gravity(&q, load_mass, &load_com, &[0.0, 0.0, -9.81]);
// g is [f64; 7] — send these torques to hold position
```

## Dynamics Compensation Diagram

```mermaid
flowchart TD
    subgraph "Robot State"
        Q["q (positions)"]
        DQ["dq (velocities)"]
        LOAD["m_load, CoM, I_load"]
    end

    subgraph "Model Computations"
        Q --> FK["pose(Frame, q)<br/>Forward Kinematics"]
        Q --> JAC["zero_jacobian(Frame, q)<br/>Jacobian"]
        Q --> M["mass(q, load)<br/>M(q)"]
        Q --> G["gravity(q, load, g_earth)<br/>g(q)"]
        Q --> C["coriolis(q, dq, load)<br/>C(q, dq)"]
        DQ --> C
        LOAD --> M
        LOAD --> G
        LOAD --> C
    end

    subgraph "Controller"
        G --> GRAV_COMP["Gravity compensation<br/>τ = g(q)"]
        G --> IMPEDANCE["Impedance control<br/>τ = g + K·(q_d - q) - D·dq"]
        JAC --> TASK_SPACE["Task-space control<br/>τ = J^T · F_task"]
        M --> COMPUTED["Computed torque<br/>τ = M·ddq_d + C·dq + g"]
        C --> COMPUTED
        G --> COMPUTED
    end
```

## Convenience Methods

Every computation has a `_from_state` variant that extracts parameters from `RobotState`:

| Method | Convenience Variant |
|--------|-------------------|
| `pose(frame, q)` | `pose_from_state(frame, &state)` |
| `body_jacobian(frame, q)` | `body_jacobian_from_state(frame, &state)` |
| `zero_jacobian(frame, q)` | `zero_jacobian_from_state(frame, &state)` |
| `mass(q, m, com, I)` | `mass_from_state(&state)` |
| `coriolis(q, dq, m, com, I)` | `coriolis_from_state(&state)` |
| `gravity(q, m, com, g)` | `gravity_from_state(&state)` |

## Example: Gravity Compensation

```rust
use franka_rs::model::Model;
use franka_rs::types::Torques;
use std::ops::ControlFlow;

let model = Model::new();

robot.control_torques(&config, |state, duration| {
    let gravity = model.gravity_from_state(state);

    if duration.as_secs_f64() >= 10.0 {
        ControlFlow::Break(Torques::new(gravity))
    } else {
        ControlFlow::Continue(Torques::new(gravity))
    }
})?;
```
