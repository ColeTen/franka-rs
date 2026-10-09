//! Integration tests for `Network::udp_send`, `Network::udp_blocking_receive`, and
//! `Network::udp_try_receive` over real loopback UDP sockets.
//!
//! `Network::connect` requires a live TCP peer, so each test spins up a minimal TCP
//! listener (accept-only, no handshake) purely to satisfy `connect`, then exercises the
//! UDP methods directly against a second, independent `UdpSocket` that stands in for the
//! robot's state-streaming peer.

use std::net::{TcpListener, UdpSocket};
use std::thread;
use std::time::Duration;

use franka_rs::errors::FrankaError;
use franka_rs::network::{Network, NetworkConfig};

/// Start a TCP listener on an ephemeral port and spawn a thread that accepts (and holds
/// open) exactly one connection. Returns the port to connect to and the listener's
/// join handle.
fn spawn_accept_only_tcp_server() -> (u16, thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let handle = thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        // Keep the connection alive for the duration of the test.
        thread::sleep(Duration::from_millis(300));
        drop(stream);
    });
    (port, handle)
}

fn test_config() -> NetworkConfig {
    NetworkConfig {
        tcp_timeout: Duration::from_secs(2),
        udp_timeout: Duration::from_millis(200),
        keepalive_enabled: false,
        ..Default::default()
    }
}

#[test]
fn udp_send_before_any_receive_errors() {
    let (port, tcp_handle) = spawn_accept_only_tcp_server();
    let network = Network::connect("127.0.0.1", port, &test_config()).unwrap();

    let result = network.udp_send(b"hello");

    assert!(result.is_err());
    match result.unwrap_err() {
        FrankaError::Network { message, .. } => {
            assert!(
                message.contains("before any state datagram"),
                "unexpected message: {message}"
            );
        }
        other => panic!("expected Network error, got: {other:?}"),
    }

    tcp_handle.join().unwrap();
}

#[test]
fn udp_blocking_receive_learns_peer_and_enables_send() {
    let (port, tcp_handle) = spawn_accept_only_tcp_server();
    let network = Network::connect("127.0.0.1", port, &test_config()).unwrap();
    let udp_port = network.udp_port();

    // Stand-in for the robot: sends one datagram to the network's UDP port, then
    // waits to receive the reply that `udp_send` routes back to it.
    let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(Duration::from_secs(2))).ok();
    peer.send_to(b"state-datagram", ("127.0.0.1", udp_port))
        .unwrap();

    let mut buf = [0u8; 64];
    let n = network.udp_blocking_receive(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"state-datagram");

    // Peer address should now be learned; udp_send should succeed and reach `peer`.
    network.udp_send(b"command").unwrap();

    let mut reply_buf = [0u8; 64];
    let (n, _) = peer.recv_from(&mut reply_buf).unwrap();
    assert_eq!(&reply_buf[..n], b"command");

    tcp_handle.join().unwrap();
}

#[test]
fn udp_try_receive_returns_none_when_no_data_available() {
    let (port, tcp_handle) = spawn_accept_only_tcp_server();
    let network = Network::connect("127.0.0.1", port, &test_config()).unwrap();

    let mut buf = [0u8; 64];
    let result = network.udp_try_receive(&mut buf).unwrap();

    assert!(result.is_none());

    tcp_handle.join().unwrap();
}

#[test]
fn udp_try_receive_returns_data_and_learns_peer() {
    let (port, tcp_handle) = spawn_accept_only_tcp_server();
    let network = Network::connect("127.0.0.1", port, &test_config()).unwrap();
    let udp_port = network.udp_port();

    let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(Duration::from_secs(2))).ok();
    peer.send_to(b"async-state", ("127.0.0.1", udp_port))
        .unwrap();

    // Give the datagram a moment to arrive so the non-blocking read observes it.
    thread::sleep(Duration::from_millis(50));

    let mut buf = [0u8; 64];
    let result = network.udp_try_receive(&mut buf).unwrap();
    let n = result.expect("expected Some(n), got None");
    assert_eq!(&buf[..n], b"async-state");

    network.udp_send(b"ack").unwrap();
    let mut reply_buf = [0u8; 64];
    let (n, _) = peer.recv_from(&mut reply_buf).unwrap();
    assert_eq!(&reply_buf[..n], b"ack");

    tcp_handle.join().unwrap();
}

#[test]
fn udp_send_targets_most_recent_sender() {
    let (port, tcp_handle) = spawn_accept_only_tcp_server();
    let network = Network::connect("127.0.0.1", port, &test_config()).unwrap();
    let udp_port = network.udp_port();

    let first_peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    let second_peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    second_peer
        .set_read_timeout(Some(Duration::from_secs(2)))
        .ok();

    first_peer
        .send_to(b"from-first", ("127.0.0.1", udp_port))
        .unwrap();
    let mut buf = [0u8; 64];
    network.udp_blocking_receive(&mut buf).unwrap();

    second_peer
        .send_to(b"from-second", ("127.0.0.1", udp_port))
        .unwrap();
    network.udp_blocking_receive(&mut buf).unwrap();

    // The learned peer should now be `second_peer`; the reply must not reach `first_peer`.
    network.udp_send(b"reply").unwrap();
    let (n, _) = second_peer.recv_from(&mut buf).unwrap();
    assert_eq!(&buf[..n], b"reply");

    first_peer
        .set_read_timeout(Some(Duration::from_millis(100)))
        .ok();
    let unexpected = first_peer.recv_from(&mut buf);
    assert!(
        unexpected.is_err(),
        "reply should not have been routed to the stale first peer"
    );

    tcp_handle.join().unwrap();
}

#[test]
fn udp_blocking_receive_times_out_with_no_data() {
    let (port, tcp_handle) = spawn_accept_only_tcp_server();
    // udp_timeout is 200ms per test_config(); no datagram is ever sent.
    let network = Network::connect("127.0.0.1", port, &test_config()).unwrap();

    let mut buf = [0u8; 64];
    let result = network.udp_blocking_receive(&mut buf);

    assert!(result.is_err());
    match result.unwrap_err() {
        FrankaError::Network { .. } => {}
        other => panic!("expected Network error, got: {other:?}"),
    }

    tcp_handle.join().unwrap();
}
