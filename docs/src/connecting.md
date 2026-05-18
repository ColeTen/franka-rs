# Connecting to the Robot

## Basic Connection

```rust
use franka_rs::robot::Robot;

let mut robot = Robot::connect("172.16.0.2")?;
```

This performs:
1. TCP connection to port 1337
2. Protocol version handshake
3. UDP socket binding for state streaming

## Custom Configuration

```rust
use franka_rs::robot::Robot;
use franka_rs::network::NetworkConfig;
use franka_rs::types::RealtimeConfig;
use std::time::Duration;

let config = NetworkConfig {
    tcp_timeout: Duration::from_secs(5),
    udp_timeout: Duration::from_millis(10),
};

let mut robot = Robot::connect_with_config("172.16.0.2", RealtimeConfig::Enforce, &config)?;
```

## Connection Lifecycle

```mermaid
stateDiagram-v2
    [*] --> Disconnected
    Disconnected --> Connecting: connect()
    Connecting --> Connected: Handshake OK
    Connecting --> Error: Version mismatch / timeout
    Connected --> Idle: Ready
    Idle --> Reading: read() / read_once()
    Idle --> Controlling: control_*()
    Reading --> Idle: Done
    Controlling --> Idle: ControlFlow::Break
    Controlling --> Error: Communication failure
    Idle --> Disconnected: Drop
    Error --> [*]
```

## Error Handling

Connection can fail in several ways:

| Error | Cause | Recovery |
|-------|-------|----------|
| `FrankaError::Network` | Robot unreachable, timeout | Check network, retry |
| `FrankaError::IncompatibleVersion` | Protocol version mismatch | Update firmware or library |
| `FrankaError::Protocol` | Malformed response | Restart robot controller |

```rust
use franka_rs::errors::FrankaError;

match Robot::connect("172.16.0.2") {
    Ok(robot) => { /* proceed */ },
    Err(FrankaError::Network { message }) => {
        eprintln!("Network error: {message}");
        eprintln!("Is the robot powered on and in FCI mode?");
    },
    Err(FrankaError::IncompatibleVersion { server, client }) => {
        eprintln!("Version mismatch: server={server}, client={client}");
    },
    Err(e) => eprintln!("Unexpected: {e}"),
}
```

## RealtimeConfig

Controls whether the control loop enforces real-time scheduling:

| Variant | Behavior |
|---------|----------|
| `Enforce` | Requires real-time thread priority (SCHED_FIFO on Linux). Fails if unavailable. |
| `Ignore` | Uses default thread priority. Suitable for development/simulation. |

For production use on a real robot, always use `Enforce` with a PREEMPT_RT kernel.
