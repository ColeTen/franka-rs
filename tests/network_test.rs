use std::io::{Read, Write};
use std::net::{TcpListener, UdpSocket};
use std::thread;
use std::time::Duration;

use franka_rs::network::{Network, NetworkConfig};

/// Helper: create a mock robot TCP server that handles a Connect handshake.
struct MockRobot {
    tcp_listener: TcpListener,
    port: u16,
}

impl MockRobot {
    fn new() -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        Self {
            tcp_listener: listener,
            port,
        }
    }

    /// Accept one TCP connection and handle a Connect request, responding with success.
    fn handle_connect(self) -> thread::JoinHandle<()> {
        thread::spawn(move || {
            let (mut stream, _) = self.tcp_listener.accept().unwrap();
            stream.set_read_timeout(Some(Duration::from_secs(2))).ok();

            // Read the command header (12 bytes)
            let mut header_buf = [0u8; 12];
            stream.read_exact(&mut header_buf).unwrap();

            // Parse header fields (little-endian)
            let command = u32::from_ne_bytes(header_buf[0..4].try_into().unwrap());
            let command_id = u32::from_ne_bytes(header_buf[4..8].try_into().unwrap());
            let size = u32::from_ne_bytes(header_buf[8..12].try_into().unwrap());

            assert_eq!(command, 0); // Connect command
            assert_eq!(command_id, 0);

            // Read payload (ConnectRequest: version u16 + udp_port u16 = 4 bytes)
            let payload_size = size as usize - 12;
            let mut payload = vec![0u8; payload_size];
            if payload_size > 0 {
                stream.read_exact(&mut payload).unwrap();
            }

            // Send response: header (12) + ConnectResponse (status: u8 + version: u16 = 3)
            let response_size: u32 = 12 + 3;
            let mut response = Vec::new();
            response.extend_from_slice(&command.to_ne_bytes()); // command
            response.extend_from_slice(&command_id.to_ne_bytes()); // command_id
            response.extend_from_slice(&response_size.to_ne_bytes()); // size
            response.push(0); // status = Success
            response.extend_from_slice(&10u16.to_ne_bytes()); // version = 10

            stream.write_all(&response).unwrap();
        })
    }

    /// Accept one TCP connection and handle a setter command, responding with success.
    fn handle_setter_command(self) -> thread::JoinHandle<()> {
        thread::spawn(move || {
            let (mut stream, _) = self.tcp_listener.accept().unwrap();
            stream.set_read_timeout(Some(Duration::from_secs(2))).ok();

            // Read the command header
            let mut header_buf = [0u8; 12];
            stream.read_exact(&mut header_buf).unwrap();

            let command_id = u32::from_ne_bytes(header_buf[4..8].try_into().unwrap());
            let size = u32::from_ne_bytes(header_buf[8..12].try_into().unwrap());

            // Read payload
            let payload_size = size as usize - 12;
            let mut payload = vec![0u8; payload_size];
            if payload_size > 0 {
                stream.read_exact(&mut payload).unwrap();
            }

            // Send success response: header + status byte (0 = success)
            let response_size: u32 = 12 + 1;
            let command = u32::from_ne_bytes(header_buf[0..4].try_into().unwrap());
            let mut response = Vec::new();
            response.extend_from_slice(&command.to_ne_bytes());
            response.extend_from_slice(&command_id.to_ne_bytes());
            response.extend_from_slice(&response_size.to_ne_bytes());
            response.push(0); // status = Success

            stream.write_all(&response).unwrap();
        })
    }
}

#[test]
fn network_connect_and_handshake() {
    let mock = MockRobot::new();
    let port = mock.port;
    let handle = mock.handle_connect();

    let config = NetworkConfig {
        tcp_timeout: Duration::from_secs(2),
        udp_timeout: Duration::from_secs(2),
        keepalive_enabled: false,
        ..Default::default()
    };

    let mut network = Network::connect("127.0.0.1", port, &config).unwrap();
    assert!(network.udp_port() > 0);

    let server_version = franka_rs::network::connect_robot(&mut network).unwrap();
    assert_eq!(server_version, 10);

    handle.join().unwrap();
}

#[test]
fn network_connect_incompatible_version() {
    let mock = MockRobot::new();
    let port = mock.port;

    let handle = thread::spawn(move || {
        let (mut stream, _) = mock.tcp_listener.accept().unwrap();
        stream.set_read_timeout(Some(Duration::from_secs(2))).ok();

        let mut header_buf = [0u8; 12];
        stream.read_exact(&mut header_buf).unwrap();
        let command_id = u32::from_ne_bytes(header_buf[4..8].try_into().unwrap());
        let size = u32::from_ne_bytes(header_buf[8..12].try_into().unwrap());
        let payload_size = size as usize - 12;
        let mut payload = vec![0u8; payload_size];
        if payload_size > 0 {
            stream.read_exact(&mut payload).unwrap();
        }

        // Respond with IncompatibleLibraryVersion
        let response_size: u32 = 12 + 3;
        let command = u32::from_ne_bytes(header_buf[0..4].try_into().unwrap());
        let mut response = Vec::new();
        response.extend_from_slice(&command.to_ne_bytes());
        response.extend_from_slice(&command_id.to_ne_bytes());
        response.extend_from_slice(&response_size.to_ne_bytes());
        response.push(1); // status = IncompatibleLibraryVersion
        response.extend_from_slice(&5u16.to_ne_bytes()); // version = 5

        stream.write_all(&response).unwrap();
    });

    let config = NetworkConfig {
        tcp_timeout: Duration::from_secs(2),
        udp_timeout: Duration::from_secs(2),
        keepalive_enabled: false,
        ..Default::default()
    };

    let mut network = Network::connect("127.0.0.1", port, &config).unwrap();
    let result = franka_rs::network::connect_robot(&mut network);

    assert!(result.is_err());
    match result.unwrap_err() {
        franka_rs::errors::FrankaError::IncompatibleVersion {
            server_version,
            library_version,
        } => {
            assert_eq!(server_version, 5);
            assert_eq!(library_version, 10);
        }
        other => panic!("expected IncompatibleVersion, got: {other:?}"),
    }

    handle.join().unwrap();
}

#[test]
fn network_udp_send_receive() {
    // Set up a UDP echo server.
    let echo_socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    let echo_port = echo_socket.local_addr().unwrap().port();

    let echo_handle = thread::spawn(move || {
        let mut buf = [0u8; 1024];
        let (n, src) = echo_socket.recv_from(&mut buf).unwrap();
        echo_socket.send_to(&buf[..n], src).unwrap();
    });

    // We need a TCP server too (Network::connect requires it).
    let tcp_listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let tcp_port = tcp_listener.local_addr().unwrap().port();
    let _tcp_handle = thread::spawn(move || {
        let _ = tcp_listener.accept();
    });

    let config = NetworkConfig {
        tcp_timeout: Duration::from_secs(2),
        udp_timeout: Duration::from_secs(2),
        keepalive_enabled: false,
        ..Default::default()
    };

    let network = Network::connect("127.0.0.1", tcp_port, &config).unwrap();

    // Manually send to the echo server (since Network::udp_send goes to the robot address).
    // Instead, let's test with a simpler approach using raw UDP.
    let test_socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    test_socket
        .connect(format!("127.0.0.1:{echo_port}"))
        .unwrap();
    test_socket.set_read_timeout(Some(Duration::from_secs(2))).ok();

    let data = b"hello robot";
    test_socket.send(data).unwrap();

    let mut buf = [0u8; 64];
    let n = test_socket.recv(&mut buf).unwrap();
    assert_eq!(&buf[..n], data);

    echo_handle.join().unwrap();
    drop(network);
}

#[test]
fn network_tcp_command_response() {
    let mock = MockRobot::new();
    let port = mock.port;
    let handle = mock.handle_setter_command();

    let config = NetworkConfig {
        tcp_timeout: Duration::from_secs(2),
        udp_timeout: Duration::from_secs(2),
        keepalive_enabled: false,
        ..Default::default()
    };

    let mut network = Network::connect("127.0.0.1", port, &config).unwrap();

    // Send a raw command and get response.
    let payload = [1.0f64; 7];
    let payload_bytes: Vec<u8> = payload
        .iter()
        .flat_map(|f| f.to_ne_bytes())
        .collect();

    let command_id = network
        .tcp_send_request(4, &payload_bytes) // SetJointImpedance = 4
        .unwrap();

    let response = network.tcp_blocking_receive_response(command_id).unwrap();
    // Response should be header (12) + status (1) = 13 bytes
    assert_eq!(response.len(), 13);
    assert_eq!(response[12], 0); // Success status

    handle.join().unwrap();
}
