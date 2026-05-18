mod command;
mod framing;
mod handshake;

pub use command::RobotCommand;
pub use handshake::{connect_gripper, connect_robot, connect_vacuum_gripper};

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{TcpStream, UdpSocket};
use std::time::Duration;

use socket2::{SockRef, TcpKeepalive};

use crate::constants;
use crate::errors::{FrankaError, FrankaResult};
use crate::wire::robot::CommandHeader;

use self::framing::TcpFraming;

/// Configuration for the network connection.
#[derive(Debug, Clone)]
pub struct NetworkConfig {
    pub tcp_timeout: Duration,
    pub udp_timeout: Duration,
    pub keepalive_enabled: bool,
    pub keepalive_idle: Duration,
    pub keepalive_interval: Duration,
}

impl Default for NetworkConfig {
    fn default() -> Self {
        Self {
            tcp_timeout: Duration::from_millis(constants::DEFAULT_TIMEOUT_MS),
            udp_timeout: Duration::from_millis(constants::DEFAULT_TIMEOUT_MS),
            keepalive_enabled: true,
            keepalive_idle: Duration::from_secs(constants::KEEPALIVE_IDLE_SECS),
            keepalive_interval: Duration::from_secs(constants::KEEPALIVE_INTERVAL_SECS),
        }
    }
}

/// Manages TCP and UDP connections to the robot.
///
/// TCP is used for command/response messages (connection setup, motion start, configuration).
/// UDP is used for high-frequency robot state and control command exchange during the control loop.
pub struct Network {
    tcp: TcpStream,
    udp: UdpSocket,
    udp_port: u16,
    next_command_id: u32,
    framing: TcpFraming,
    received_responses: HashMap<u32, Vec<u8>>,
}

impl Network {
    /// Connect to a robot at the given address and port.
    pub fn connect(address: &str, port: u16, config: &NetworkConfig) -> FrankaResult<Self> {
        let tcp_addr = format!("{address}:{port}");
        let tcp = TcpStream::connect_timeout(
            &tcp_addr.parse().map_err(|e| {
                FrankaError::network(format!("invalid address '{tcp_addr}': {e}"))
            })?,
            config.tcp_timeout,
        )
        .map_err(|e| FrankaError::network_with_source(format!("TCP connect to {tcp_addr}"), e))?;

        tcp.set_read_timeout(Some(config.tcp_timeout))
            .map_err(|e| FrankaError::network_with_source("set TCP read timeout", e))?;
        tcp.set_write_timeout(Some(config.tcp_timeout))
            .map_err(|e| FrankaError::network_with_source("set TCP write timeout", e))?;
        tcp.set_nodelay(true)
            .map_err(|e| FrankaError::network_with_source("set TCP_NODELAY", e))?;

        if config.keepalive_enabled {
            let sock_ref = SockRef::from(&tcp);
            let keepalive = TcpKeepalive::new()
                .with_time(config.keepalive_idle)
                .with_interval(config.keepalive_interval);
            sock_ref
                .set_tcp_keepalive(&keepalive)
                .map_err(|e| FrankaError::network_with_source("set TCP keepalive", e))?;
        }

        // Bind UDP socket to any available port.
        let udp = UdpSocket::bind("0.0.0.0:0")
            .map_err(|e| FrankaError::network_with_source("bind UDP socket", e))?;
        udp.set_read_timeout(Some(config.udp_timeout))
            .map_err(|e| FrankaError::network_with_source("set UDP read timeout", e))?;

        // Connect UDP to the robot's address so we can use send/recv instead of send_to/recv_from.
        let udp_target = format!("{address}:{port}");
        udp.connect(&udp_target)
            .map_err(|e| FrankaError::network_with_source("connect UDP socket", e))?;

        let udp_port = udp
            .local_addr()
            .map_err(|e| FrankaError::network_with_source("get UDP local port", e))?
            .port();

        Ok(Self {
            tcp,
            udp,
            udp_port,
            next_command_id: 0,
            framing: TcpFraming::new(),
            received_responses: HashMap::new(),
        })
    }

    /// Returns the local UDP port that the robot should send state to.
    pub fn udp_port(&self) -> u16 {
        self.udp_port
    }

    /// Send a TCP request and return the assigned command ID.
    pub fn tcp_send_request(&mut self, command: u32, payload: &[u8]) -> FrankaResult<u32> {
        let command_id = self.next_command_id;
        self.next_command_id += 1;

        let total_size = (CommandHeader::SIZE + payload.len()) as u32;
        let header = CommandHeader {
            command,
            command_id,
            size: total_size,
        };

        let header_bytes = header.to_bytes();
        self.tcp
            .write_all(&header_bytes)
            .map_err(|e| FrankaError::network_with_source("TCP send header", e))?;

        if !payload.is_empty() {
            self.tcp
                .write_all(payload)
                .map_err(|e| FrankaError::network_with_source("TCP send payload", e))?;
        }

        Ok(command_id)
    }

    /// Block until a response with the given command ID is received.
    /// Returns the full response message (header + payload bytes).
    pub fn tcp_blocking_receive_response(&mut self, command_id: u32) -> FrankaResult<Vec<u8>> {
        loop {
            // Check if we already have the response buffered.
            if let Some(response) = self.received_responses.remove(&command_id) {
                return Ok(response);
            }

            // Read more data from TCP.
            self.tcp_read_message()?;
        }
    }

    /// Try to receive a response with the given command ID (non-blocking).
    /// Returns None if no matching response is available yet.
    pub fn tcp_try_receive_response(&mut self, command_id: u32) -> FrankaResult<Option<Vec<u8>>> {
        if let Some(response) = self.received_responses.remove(&command_id) {
            return Ok(Some(response));
        }

        // Do a non-blocking read attempt.
        self.tcp.set_nonblocking(true).ok();
        let result = self.tcp_try_read_message();
        self.tcp.set_nonblocking(false).ok();

        match result {
            Ok(()) => Ok(self.received_responses.remove(&command_id)),
            Err(e) => Err(e),
        }
    }

    /// Send data over UDP.
    pub fn udp_send(&self, data: &[u8]) -> FrankaResult<()> {
        let sent = self
            .udp
            .send(data)
            .map_err(|e| FrankaError::network_with_source("UDP send", e))?;
        if sent != data.len() {
            return Err(FrankaError::network(format!(
                "UDP send: sent {sent} bytes, expected {}",
                data.len()
            )));
        }
        Ok(())
    }

    /// Blocking receive from UDP. Returns the received bytes.
    pub fn udp_blocking_receive(&self, buf: &mut [u8]) -> FrankaResult<usize> {
        let received = self
            .udp
            .recv(buf)
            .map_err(|e| FrankaError::network_with_source("UDP receive", e))?;
        Ok(received)
    }

    /// Non-blocking receive from UDP. Returns None if no data available.
    pub fn udp_try_receive(&self, buf: &mut [u8]) -> FrankaResult<Option<usize>> {
        self.udp.set_nonblocking(true).ok();
        let result = self.udp.recv(buf);
        self.udp.set_nonblocking(false).ok();

        match result {
            Ok(n) => Ok(Some(n)),
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(FrankaError::network_with_source("UDP receive", e)),
        }
    }

    /// Check if TCP connection is still alive.
    pub fn is_tcp_alive(&self) -> bool {
        let sock_ref = SockRef::from(&self.tcp);
        sock_ref.take_error().map(|e| e.is_none()).unwrap_or(false)
    }

    // --- Internal helpers ---

    /// Read one complete message from TCP and buffer it.
    fn tcp_read_message(&mut self) -> FrankaResult<()> {
        // Read header if we don't have one pending.
        if !self.framing.has_pending_header() {
            let mut header_buf = [0u8; CommandHeader::SIZE];
            self.tcp
                .read_exact(&mut header_buf)
                .map_err(|e| FrankaError::network_with_source("TCP read header", e))?;
            self.framing.set_header(&header_buf)?;
        }

        // Read remaining payload.
        while !self.framing.is_complete() {
            let remaining = self.framing.remaining_bytes();
            let mut chunk = vec![0u8; remaining.min(4096)];
            let n = self
                .tcp
                .read(&mut chunk)
                .map_err(|e| FrankaError::network_with_source("TCP read payload", e))?;
            if n == 0 {
                return Err(FrankaError::network("server closed connection"));
            }
            self.framing.push_bytes(&chunk[..n]);
        }

        let (cmd_id, message) = self.framing.take_message();
        self.received_responses.insert(cmd_id, message);
        Ok(())
    }

    /// Non-blocking attempt to read a message.
    fn tcp_try_read_message(&mut self) -> FrankaResult<()> {
        if !self.framing.has_pending_header() {
            let mut header_buf = [0u8; CommandHeader::SIZE];
            match self.tcp.read_exact(&mut header_buf) {
                Ok(()) => self.framing.set_header(&header_buf)?,
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
                Err(e) => return Err(FrankaError::network_with_source("TCP read header", e)),
            }
        }

        while !self.framing.is_complete() {
            let remaining = self.framing.remaining_bytes();
            let mut chunk = vec![0u8; remaining.min(4096)];
            match self.tcp.read(&mut chunk) {
                Ok(0) => return Err(FrankaError::network("server closed connection")),
                Ok(n) => self.framing.push_bytes(&chunk[..n]),
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
                Err(e) => return Err(FrankaError::network_with_source("TCP read payload", e)),
            }
        }

        let (cmd_id, message) = self.framing.take_message();
        self.received_responses.insert(cmd_id, message);
        Ok(())
    }
}

impl Drop for Network {
    fn drop(&mut self) {
        let _ = self.tcp.shutdown(std::net::Shutdown::Both);
    }
}
