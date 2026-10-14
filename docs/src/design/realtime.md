# Real-Time Considerations

Every millisecond the robot sends a state, and your application must send the command answering
it. Late commands are missed; `RobotState::control_command_success_rate` (0–1) reports the share
received in time, and the robot's `COMMUNICATION_CONSTRAINTS_VIOLATION` flag reports
communication problems (the robot sets the threshold).

## The Cycle

In the callback interface each cycle receives the newest state, runs your callback, filters,
limits and checks the command, and sends it. Your callback shares the 1 ms; franka-rs's own
processing time has not been measured, so measure your cycle on your system.

| Avoid in the callback | Instead |
|-----------------------|---------|
| `println!`, logging, file or network I/O | Buffer, and write after control ends |
| Heap allocation | Pre-allocate before the loop |
| Mutex locking | Lock-free channels or atomics |

## Real-Time Scheduling

franka-rs stores `RealtimeConfig` but does not act on it: it sets no thread priority and does not
check the kernel (libfranka's default `kEnforce` does both, and refuses to connect without a
real-time kernel). Set up scheduling for your control thread yourself.

General Linux guidance, not verified with franka-rs: a PREEMPT_RT kernel; real-time priority and
locked memory for your user (`/etc/security/limits.conf`: `rtprio 99`, `memlock unlimited`); a
real-time scheduling policy for the control thread; optionally an isolated core (`isolcpus`,
`nohz_full`, `rcu_nocbs`) and the `performance` CPU governor. Use wired Ethernet.
