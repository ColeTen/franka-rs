# Reading Robot State

The robot sends a `RobotState` every millisecond over UDP; values arrive in single precision and
are widened to `f64`.

All joint arrays are `[f64; 7]`; poses are `[f64; 16]` column-major 4×4 matrices; wrenches and
twists are `[f64; 6]` (linear, then angular).

**Joints**

| Field | Meaning |
|-------|---------|
| `q`, `dq` | Measured positions (rad) and velocities (rad/s) |
| `q_d`, `dq_d`, `ddq_d` | Desired positions, velocities, accelerations |
| `theta`, `dtheta` | Motor positions (rad) and velocities (rad/s) |
| `tau_j`, `dtau_j` | Measured link-side torques (Nm) and their derivative (Nm/s) |
| `tau_j_d` | Desired torques, without gravity (Nm) |
| `tau_ext_hat_filtered` | Filtered external torque estimate (Nm) |
| `joint_contact`, `joint_collision` | Contact levels (0 = none); collision levels (kept until reset) |

**Cartesian**

| Field | Meaning |
|-------|---------|
| `o_t_ee`, `o_t_ee_d`, `o_t_ee_c` | End-effector pose in the base frame: measured, desired, last commanded |
| `o_dp_ee_d`, `o_dp_ee_c`, `o_ddp_ee_c` | End-effector twist desired and commanded; commanded acceleration |
| `o_ddp_o` | Base linear acceleration (`[f64; 3]`) |
| `o_f_ext_hat_k`, `k_f_ext_hat_k` | External wrench estimate in the base / stiffness frame (N, Nm) |
| `cartesian_contact`, `cartesian_collision` | Contact and collision levels (x, y, z, R, P, Y) |
| `elbow`, `elbow_d` | Elbow (`[f64; 2]`: joint-3 angle, joint-4 sign): measured, desired |
| `elbow_c`, `delbow_c`, `ddelbow_c` | Commanded elbow, its velocity and acceleration |

**Frames and loads**

| Field | Meaning |
|-------|---------|
| `f_t_ee`, `f_t_ne`, `ne_t_ee`, `ee_t_k` | F_T_EE, F_T_NE, NE_T_EE, EE_T_K (F flange, NE nominal end effector, EE end effector, K stiffness) |
| `m_ee`, `f_x_cee`, `i_ee` | End effector: mass (kg), center of mass in the flange frame (`[f64; 3]`), inertia (`[f64; 9]`, column-major) |
| `m_load`, `f_x_cload`, `i_load` | External load, same layout |

**Status**

| Field | Type | Meaning |
|-------|------|---------|
| `robot_mode` | `RobotMode` | Current mode |
| `motion_generator_mode` | `MotionGeneratorMode` | Running motion generator |
| `current_errors`, `last_motion_errors` | `RobotErrors` | Active errors; errors that ended the last motion |
| `control_command_success_rate` | `f64` | Share of the last 100 commands received in time (0–1) |
| `time` | `Duration` | Robot time (the state's message ID, ms) |

## Reading

```rust
let state = robot.read_once()?;   // newest state if newer than the last read, else waits
println!("z = {:.4} m", state.o_t_ee[14]);

robot.read(|state| {              // until the callback returns false
    println!("tau_ext: {:?}", state.tau_ext_hat_filtered);
    state.robot_mode == RobotMode::Idle
})?;
```

## With the Model

```rust
let model = robot.load_model()?;  // from the robot's URDF; needs `use franka_rs::model::RobotModel`
let state = robot.read_once()?;
let ee_pose = model.pose_from_state(Frame::EndEffector, &state);           // Isometry3
let jacobian = model.zero_jacobian_from_state(Frame::EndEffector, &state); // 6×7
let gravity = model.gravity_from_state(&state);  // for analysis; the robot compensates gravity itself
```
