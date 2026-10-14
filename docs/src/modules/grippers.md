# Gripper & Vacuum Gripper

> **Not validated on hardware.** The gripper protocols' 10-byte header (16-bit command) is tested
> against a local fake gripper only, not against a real gripper or libfranka.

Usage examples: [Gripper Interface](../gripper.md).

## `Gripper` (TCP port 1338)

| Method | Returns |
|--------|---------|
| `connect(address)` | `FrankaResult<Gripper>` |
| `server_version()` | `u16` |
| `homing()` | `FrankaResult<bool>` |
| `grasp(width, speed, force, epsilon_inner, epsilon_outer)` | `FrankaResult<bool>` |
| `move_fingers(width, speed)` | `FrankaResult<bool>` |
| `stop()` | `FrankaResult<bool>` |
| `read_once()` | `FrankaResult<GripperState>` |

`GripperState`: `width`, `max_width` (m, from homing), `is_grasped: bool`, `temperature: u16`
(°C). A grasp succeeds when the measured width `d` satisfies
`width − epsilon_inner < d < width + epsilon_outer`.

## `VacuumGripper` (TCP port 1339)

| Method | Returns |
|--------|---------|
| `connect(address)` | `FrankaResult<VacuumGripper>` |
| `server_version()` | `u16` |
| `vacuum(setpoint, timeout, profile)` | `FrankaResult<bool>` — setpoint in units of 10 mbar |
| `drop_off(timeout)` | `FrankaResult<bool>` |
| `stop()` | `FrankaResult<bool>` |
| `read_once()` | `FrankaResult<VacuumGripperState>` |

`VacuumGripperState`: `in_control_range`, `part_detached`, `part_present` (`bool`),
`device_status: VacuumDeviceStatus` (`Green`, `Yellow`, `Orange`, `Red`; an unknown status reads
as `Red`), `actual_power` (%), `vacuum` (mbar). `VacuumProfile::P0`–`P3` select the device's
production setup profiles, as configured on the device.

## Results

`Ok(true)` on success; `Ok(false)` when the device reports the command unsuccessful;
`FrankaError::Command` when it reports failure or abort; `FrankaError::Protocol` for an unknown
status. `read_once` waits for the next UDP state.
