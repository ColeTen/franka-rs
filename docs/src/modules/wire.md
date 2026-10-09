# Wire Protocol

`wire` (crate-private) holds `#[repr(C, packed)]` structs matching the FCI protocol byte for byte,
for zero-copy encoding and decoding. For `wire::robot`, sizes, field offsets and enum values are
checked against libfranka's headers (`validation/data/message_layout.txt`); the gripper and vacuum
structs are not checked. `logging::LogEntry::command` exposes a `wire::robot::RobotCommand`
(distinct from `network::RobotCommand`, the TCP command helper).

## `wire::robot`

| Struct | Channel | Purpose |
|--------|---------|---------|
| `CommandHeader` | TCP | 12 bytes: `command`, `command_id`, `size` (`u32` each) |
| `ConnectRequest` / `ConnectResponse` | TCP | Handshake (version, UDP port) |
| `MoveRequest` | TCP | Start a motion (controller and motion generator modes, deviations) |
| `SetCollisionBehaviorRequest`, `SetJointImpedanceRequest`, `SetCartesianImpedanceRequest`, `SetGuidingModeRequest`, `SetLoadRequest`, `SetNeToEeRequest`, `SetEeToKRequest` | TCP | Configuration |
| `CommandResponse` | TCP | Setter status |
| `RawRobotState` | UDP | Robot state, 1377 bytes |
| `RobotCommand` = `message_id` + `MotionGeneratorCommand` + `ControllerCommand` | UDP | Command, 371 bytes |

Enums: `Command`, `ConnectStatus`, `MoveControllerMode` and `MoveMotionGeneratorMode` (the Move
request numbering, which differs from the robot state's), `MoveStatus`, `GetterSetterStatus`,
`StopMoveStatus`, `AutomaticErrorRecoveryStatus`.

## `wire::gripper` and `wire::vacuum`

| Struct | Purpose |
|--------|---------|
| `CommandHeader` | TCP header, 10 bytes: `command: u16`, `command_id: u32`, `size: u32` |
| `RawGripperState` / `RawVacuumGripperState` | Gripper state (UDP) |
| `GraspRequest`, `MoveRequest` / `VacuumRequest`, `DropOffRequest` | Command parameters |

## Robot State Layout

`RawRobotState`, packed, in order: `message_id: u64` (also the time, ms); poses `f32[16]`
(`O_T_EE`, `O_T_EE_d`, `F_T_EE`, `EE_T_K`, `F_T_NE`, `NE_T_EE`); end effector and load mass,
inertia, center of mass; `elbow`, `elbow_d`; joint `f32[7]` fields; contact and collision;
external torque and wrenches; `O_dP_EE_d`, `O_ddP_O`; `elbow_c`, `delbow_c`, `ddelbow_c`;
`O_T_EE_c`, `O_dP_EE_c`, `O_ddP_EE_c`; motor positions and velocities; accelerometers
`f32[3] × 6`, top and bottom; `motion_generator_mode`, `controller_mode` (`u8`); `errors`, `reflex_reason`
(`u8[41]`); `robot_mode` (`u8`); `control_command_success_rate` (`f32`).

## `RobotCommand` Layout

| Part | Fields |
|------|--------|
| `message_id` | `u64` — the ID of the state this command answers |
| `MotionGeneratorCommand` | `q_c`, `dq_c` (`f64[7]`), `o_t_ee_c` (`f64[16]`), `o_dp_ee_c` (`f64[6]`), `elbow_c` (`f64[2]`), `valid_elbow`, `motion_generation_finished` (`u8`) |
| `ControllerCommand` | `tau_j_d` (`f64[7]`), `torque_command_finished` (`u8`) |

## Conversion to Public Types

| Wire | Public | Conversion |
|------|--------|------------|
| `f32` arrays | `f64` arrays | widening |
| `u8` flags / enums | `bool` / Rust enums | `!= 0` / `from_wire` |
| `u8[41]` | `RobotErrors` | `from_bool_array` |
| `message_id` (ms) | `RobotState::time` | `Duration::from_millis` |

Decoding is `unsafe` (byte reinterpretation) and sound because every field is numeric, the
structs have no padding, and the size is checked first: a robot state must be exactly
`RawRobotState::SIZE` bytes (the receive buffer is one byte larger to detect oversized datagrams);
gripper states must be at least their struct size.
