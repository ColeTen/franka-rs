# Real-Time Considerations

## The 1 kHz Constraint

The Franka robot runs its control loop at 1 kHz (1 ms cycle). Every millisecond:

1. The robot sends its current state over UDP
2. Your application must compute and send a command answering that state
3. The robot applies the command

Commands that arrive late are missed; the share of recent commands received in time is reported in
`RobotState::control_command_success_rate`. The robot's error flags include
`COMMUNICATION_CONSTRAINTS_VIOLATION` for communication problems (the threshold is set by the
robot and not documented here).

## What Runs in a Cycle

In the callback interface, each cycle franka-rs receives the newest state, calls your callback,
low-pass filters and rate-limits the command (as configured in `MotionConfig`), checks it, and
sends it. Your callback's execution time is part of the 1 ms budget:

```rust
robot.control_torques(&config, |state, _period| {
    // Keep this short: it shares the 1 ms cycle with receiving, processing and sending.
    ControlFlow::Continue(Torques::new(compute(state)))
})?;
```

No timing of franka-rs's own processing has been measured; measure your callback and the whole
cycle on your system (for example with `std::time::Instant`, outside the measured path).

### What to Avoid in the Callback

| Operation | Risk | Alternative |
|-----------|------|-------------|
| `println!` / logging | Blocks on I/O | Write to a pre-allocated buffer, log after control ends |
| Heap allocation (`Vec::new`) | Allocator latency | Pre-allocate before the loop |
| File I/O, network calls | Unbounded latency | Not in the loop |
| Mutex locking | Priority inversion | Lock-free channels or atomics |

## `RealtimeConfig`

franka-rs stores `RealtimeConfig` (`Robot::connect` uses `Ignore`;
`Robot::connect_with_config(address, RealtimeConfig::Enforce)` stores `Enforce`) and returns it
from `Robot::realtime_config()`, but does not act on it: it sets no thread priority and does not
check the kernel. libfranka, with its default `kEnforce`, sets the highest scheduler priority and
refuses to connect without a real-time kernel. With franka-rs, set up real-time scheduling for
your control thread yourself.

## Linux Real-Time Setup

General Linux guidance (not specific to franka-rs, and not verified with it): for control on a real
robot, use a real-time (PREEMPT_RT) kernel, allow your user real-time
priority and locked memory (for example in `/etc/security/limits.conf`: `rtprio 99`,
`memlock unlimited` for your group), and run the control thread with a real-time scheduling
policy. Optionally isolate a CPU core for the control thread (kernel parameters `isolcpus`,
`nohz_full`, `rcu_nocbs`) and set the CPU frequency governor to `performance`.

## Communication Monitoring

```rust
let state = robot.read_once()?;
println!("Success rate: {:.1}%", state.control_command_success_rate * 100.0);
```

`control_command_success_rate` is the fraction (0–1) of recent control commands the robot
received in time; a falling value means commands are arriving late or are lost.

## Tips

1. **Measure your callback time**, and keep it well under 1 ms
2. **Pre-compute and pre-allocate** before the control loop
3. **Use wired Ethernet**, not WiFi
4. **Monitor `control_command_success_rate`**
