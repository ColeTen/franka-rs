# Live Network Comparison

Records the traffic of a libfranka example and its franka-rs counterpart with `tcpdump`, for
manual comparison. The automated comparisons — A1 against a mock robot (`validation/a1/run_a1.py`)
and A2 on the robot (`validation/a2/run_a2.py`) — are described in
`validation/torque_validation_plan.md`.

| Operation | libfranka | franka-rs | Moves the robot |
|-----------|-----------|-----------|-----------------|
| Read 101 states | `echo_robot_state` | `examples/echo_robot_state.rs` | No |
| Move to a start pose, then 10 s of zero torque | `communication_test` | `examples/communication_test.rs` | **Yes** |

Both connect and download the URDF first, so each recording also covers the handshake. Run the
read-only comparison first, and one program at a time (the robot accepts one client).

**Needs:** robot on, joints unlocked, FCI active; `tcpdump` with `sudo`. Run from the repository
root; `<robot-ip>` is the robot's address, `<iface>` the interface reaching it
(`ip route get <robot-ip>` shows it after `dev`).

## Build

```sh
cmake -S external/libfranka -B external/libfranka/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON -DBUILD_TESTS=OFF
cmake --build external/libfranka/build -j"$(nproc)"
cargo build --release --example echo_robot_state --example communication_test
mkdir -p validation/captures
```

Run the built binaries directly (not `cargo run`), so the recordings contain only their traffic.

## Record

For each `<program>` (`echo_robot_state`, `communication_test`) and library, start `tcpdump` in one
terminal and, once it prints `listening on`, run the program in another; stop `tcpdump` with
`Ctrl+C` when the program has finished (`Done.`, or the `#####` summary).

```sh
# terminal 1 (name: <program>_libfranka or <program>_franka_rs)
sudo tcpdump -i <iface> -Z "$USER" -w validation/captures/<name>.pcap host <robot-ip>

# terminal 2: libfranka, then franka-rs
./external/libfranka/build/examples/<program> <robot-ip> | tee validation/captures/<program>_libfranka.txt
./target/release/examples/<program> <robot-ip> | tee validation/captures/<program>_franka_rs.txt
```

`-Z "$USER"` writes the file as your user; `host <robot-ip>` is used because the UDP port is chosen
at connect time.

> **Warning:** `communication_test` moves the robot to `[0, -π/4, 0, -3π/4, 0, π/2, π/4]` at half
> speed and then holds it with zero torque for 10 s. Clear the space, keep the user stop button in
> hand; it waits for Enter before moving.

## Inspect

```sh
tcpdump -nn -tt -r validation/captures/<name>.pcap                    # all packets
tcpdump -nn -X -r validation/captures/<name>.pcap 'tcp port 1337'     # command exchange, hex
tcpdump -nn -r validation/captures/<name>.pcap udp | grep -o 'length [0-9]*' | sort | uniq -c
```

Matching recordings have the same TCP commands in the same order with the same sizes. Robot states
are 1377 bytes and commands 371; `echo_robot_state` sends no commands, `communication_test` one per
state during the motion.
