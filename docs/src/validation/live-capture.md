# Live Network Comparison

This page describes how to compare the network traffic franka-rs exchanges with a real Franka
Research 3 against the traffic libfranka exchanges for the same operation. Each operation is run
twice, once with a libfranka example program and once with its franka-rs counterpart, while
`tcpdump` records the traffic. The two recordings are then compared.

| Operation | libfranka program | franka-rs program | Moves the robot |
|-----------|-------------------|-------------------|-----------------|
| Read 101 robot states | `echo_robot_state` | `examples/echo_robot_state.rs` | No |
| Move to a start pose, then 10 s of zero-torque control | `communication_test` | `examples/communication_test.rs` | **Yes** |

Both programs connect, download the robot's URDF (the `GetRobotModel` command), and then run their
operation, so each recording also covers the connection handshake and the URDF download.

Run the read-only comparison first. Run the motion comparison only after the read-only recordings
match.

## Prerequisites

- The robot is powered on, its joints are unlocked, and FCI is enabled.
- `tcpdump` is installed and you can run it with `sudo`.
- No other program is connected to the robot. The robot accepts one client at a time, so run the
  libfranka and franka-rs programs one after the other, never together.

All commands below run from the repository root. Replace `<robot-ip>` with the robot's numeric IP
address and `<iface>` with the network interface that reaches it.

## 1. Build the programs

The libfranka examples are built with libfranka itself:

```sh
cmake -S external/libfranka -B external/libfranka/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF
cmake --build external/libfranka/build -j"$(nproc)"
```

The franka-rs programs:

```sh
cargo build --release --example echo_robot_state --example communication_test
```

This produces:

- `external/libfranka/build/examples/echo_robot_state` and `.../communication_test`
- `target/release/examples/echo_robot_state` and `.../communication_test`

The compiled programs are run directly rather than through `cargo run`, so the recordings contain
only the program's own traffic.

## 2. Find the interface that reaches the robot

```sh
ip route get <robot-ip>
```

The interface name follows `dev` in the output (for example `dev enp3s0`).

## 3. Read-only comparison (`echo_robot_state`)

Create the directory for the recordings:

```sh
mkdir -p validation/captures
```

Each run uses two terminals: the first records, the second runs the program.

### libfranka

Terminal 1, start recording:

```sh
sudo tcpdump -i <iface> -Z "$USER" -w validation/captures/echo_libfranka.pcap host <robot-ip>
```

Terminal 2, once `tcpdump` prints `listening on <iface>`:

```sh
./external/libfranka/build/examples/echo_robot_state <robot-ip> | tee validation/captures/echo_libfranka.txt
```

When it finishes (its last line is `Done.`), stop the recording in terminal 1 with `Ctrl+C`.

### franka-rs

Terminal 1:

```sh
sudo tcpdump -i <iface> -Z "$USER" -w validation/captures/echo_franka_rs.pcap host <robot-ip>
```

Terminal 2:

```sh
./target/release/examples/echo_robot_state <robot-ip> | tee validation/captures/echo_franka_rs.txt
```

Stop the recording with `Ctrl+C` when it finishes.

`-Z "$USER"` makes `tcpdump` write the file as your user rather than as root, so the recordings can
be read without `sudo`. The filter `host <robot-ip>` is used rather than a port filter because the
UDP port for robot states is chosen when the program connects.

## 4. Motion comparison (`communication_test`)

> **Warning:** both programs move the robot to the joint configuration
> `[0, -π/4, 0, -3π/4, 0, π/2, π/4]` at half speed, then hold it with zero torque for 10 s.
> Clear the space around the robot and keep the user stop button in hand. Each program waits for
> Enter before it moves.

### libfranka

Terminal 1:

```sh
sudo tcpdump -i <iface> -Z "$USER" -w validation/captures/communication_libfranka.pcap host <robot-ip>
```

Terminal 2:

```sh
./external/libfranka/build/examples/communication_test <robot-ip> | tee validation/captures/communication_libfranka.txt
```

Press Enter when prompted. Stop the recording with `Ctrl+C` after the summary block (between the
`#####` lines) is printed.

### franka-rs

Terminal 1:

```sh
sudo tcpdump -i <iface> -Z "$USER" -w validation/captures/communication_franka_rs.pcap host <robot-ip>
```

Terminal 2:

```sh
./target/release/examples/communication_test <robot-ip> | tee validation/captures/communication_franka_rs.txt
```

Press Enter when prompted. Stop the recording with `Ctrl+C` after the summary block is printed.

## 5. Inspect the recordings

List every packet with its direction, protocol, and length:

```sh
tcpdump -nn -tt -r validation/captures/echo_libfranka.pcap
tcpdump -nn -tt -r validation/captures/echo_franka_rs.pcap
```

Show only the command exchange (TCP port 1337), with the payload bytes in hexadecimal:

```sh
tcpdump -nn -X -r validation/captures/echo_libfranka.pcap 'tcp port 1337'
tcpdump -nn -X -r validation/captures/echo_franka_rs.pcap 'tcp port 1337'
```

Count UDP packets by length (robot states are 1377 bytes; robot commands are 371 bytes):

```sh
tcpdump -nn -r validation/captures/echo_libfranka.pcap udp | grep -o 'length [0-9]*' | sort | uniq -c
tcpdump -nn -r validation/captures/echo_franka_rs.pcap udp | grep -o 'length [0-9]*' | sort | uniq -c
```

The same commands apply to the `communication_*.pcap` recordings.

In matching recordings, the TCP commands appear in the same order with the same command numbers
and sizes. For `echo_robot_state`, the program sends no UDP commands. For `communication_test`,
the program sends one 371-byte UDP command for each robot state it receives during the motion.
