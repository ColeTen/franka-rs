# Connecting to the Robot

```rust
let mut robot = Robot::connect("172.16.0.2")?;
```

`connect`:
1. opens TCP to port 1337 (keepalive, 1 s read/write timeout);
2. binds a UDP socket on an ephemeral port for the state stream;
3. performs the version handshake (the Connect request carries the UDP port);
4. waits for the first robot state;
5. downloads the URDF (GetRobotModel) and reads the joint velocity limits (a mobile robot's URDF,
   robot name starting with `tmr`, keeps all-zero limits).

The address must be an IP address (for example `172.16.0.2`); hostnames are not resolved.

Network settings are `NetworkConfig::default()` and cannot be changed through `Robot`.

## Errors

| Error | Cause |
|-------|-------|
| `Network` | Robot unreachable, timeout |
| `IncompatibleVersion` | Protocol version mismatch (update firmware or library) |
| `Protocol` | Malformed response |
| `Command` | The robot rejected GetRobotModel |
| `Model` | The URDF lacks valid joint velocity limits |

```rust
match Robot::connect("172.16.0.2") {
    Ok(robot) => { /* proceed */ }
    Err(FrankaError::Network { message, .. }) => eprintln!("Network: {message} (robot on, FCI active?)"),
    Err(FrankaError::IncompatibleVersion { server_version, library_version }) => {
        eprintln!("Version mismatch: server {server_version}, library {library_version}")
    }
    Err(e) => eprintln!("{e}"),
}
```

## Real-Time Scheduling

`Robot::connect_with_config(address, RealtimeConfig::Enforce | Ignore)` stores the setting
(`connect` uses `Ignore`), but franka-rs does not act on it: it sets no thread priority and does
not check the kernel. libfranka's default `kEnforce` sets the highest scheduler priority and
refuses to connect without a real-time kernel. Set up real-time scheduling for your control
thread yourself; see [Real-Time Considerations](./design/realtime.md).
