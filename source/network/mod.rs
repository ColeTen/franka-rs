mod command;
mod framing;
mod handshake;
mod header;

pub use command::RobotCommand;
pub use handshake::{connect_gripper, connect_robot, connect_vacuum_gripper};

use std::cell::Cell;
use std::collections::HashMap;
use std::io::{self, Read, Write};
use std::mem::MaybeUninit;
use std::net::{IpAddr, SocketAddr, TcpStream, UdpSocket};
use std::os::fd::AsRawFd;
use std::time::Duration;

use socket2::{SockRef, TcpKeepalive};

use crate::constants;
use crate::errors::{FrankaError, FrankaResult};
use crate::wire::robot::CommandHeader;

use self::framing::TcpFraming;
use self::header::MessageHeader;

/// Size of the stack buffer header bytes are read into; at least every protocol's header size.
const HEADER_BUFFER_SIZE: usize = 16;

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
/// TCP messages start with the header `H` of the protocol spoken: the robot's by default, the
/// gripper's or vacuum gripper's for those devices.
pub struct Network<H = CommandHeader> {
    tcp: TcpStream,
    udp: UdpSocket,
    udp_port: u16,
    /// The robot's IP address. UDP datagrams from any other source are ignored so that a
    /// stray or hostile host cannot redirect outgoing control commands.
    robot_ip: IpAddr,
    /// Source address of the most recent UDP datagram from the robot.
    ///
    /// The robot streams state from an ephemeral port, not the command port, so the
    /// socket is left unconnected and outgoing datagrams are sent back to whichever
    /// address the incoming state arrived from. `None` until the first datagram is received.
    udp_peer: Cell<Option<SocketAddr>>,
    /// Message ID of the most recently received robot state. Outgoing robot commands carry this
    /// ID so the robot can match each command to the state it answers. Zero until a state arrives.
    latest_state_message_id: Cell<u64>,
    /// Motion generator and controller modes (robot-state numbering) of the most recently received
    /// robot state. Idle and Other until a state arrives.
    latest_state_modes: Cell<(u8, u8)>,
    /// Robot mode (robot-state numbering) of the most recently received robot state.
    latest_robot_mode: Cell<u8>,
    /// Motion generator and controller modes (robot-state numbering) the running Move requested.
    current_move_modes: Cell<Option<(u8, u8)>>,
    next_command_id: u32,
    framing: TcpFraming<H>,
    received_responses: HashMap<u32, Vec<u8>>,
}

impl Network {
    /// Connect to a robot at the given address and port.
    pub fn connect(address: &str, port: u16, config: &NetworkConfig) -> FrankaResult<Self> {
        Self::connect_with_header(address, port, config)
    }
}

impl<H: MessageHeader> Network<H> {
    /// Connect to a device at the given address and port whose TCP messages use the header `H`.
    pub fn connect_with_header(address: &str, port: u16, config: &NetworkConfig) -> FrankaResult<Self> {
        let tcp_addr = format!("{address}:{port}");
        let robot_addr: SocketAddr = tcp_addr
            .parse()
            .map_err(|e| FrankaError::network(format!("invalid address '{tcp_addr}': {e}")))?;
        let tcp = TcpStream::connect_timeout(&robot_addr, config.tcp_timeout)
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

        // Bind UDP socket to any available port, matching the robot's address family so
        // that IPv6 robots work too. The socket is intentionally left unconnected: the
        // robot streams state from an ephemeral source port rather than the command port,
        // so a connected socket would drop every state datagram.
        let udp_bind_addr = match robot_addr.ip() {
            IpAddr::V4(_) => "0.0.0.0:0",
            IpAddr::V6(_) => "[::]:0",
        };
        let udp = UdpSocket::bind(udp_bind_addr)
            .map_err(|e| FrankaError::network_with_source("bind UDP socket", e))?;
        udp.set_read_timeout(Some(config.udp_timeout))
            .map_err(|e| FrankaError::network_with_source("set UDP read timeout", e))?;

        let udp_port = udp
            .local_addr()
            .map_err(|e| FrankaError::network_with_source("get UDP local port", e))?
            .port();

        Ok(Self {
            tcp,
            udp,
            udp_port,
            robot_ip: robot_addr.ip(),
            udp_peer: Cell::new(None),
            latest_state_message_id: Cell::new(0),
            latest_state_modes: Cell::new((
                crate::types::MotionGeneratorMode::Idle as u8,
                crate::wire::robot::CONTROLLER_MODE_OTHER,
            )),
            latest_robot_mode: Cell::new(crate::types::RobotMode::Other as u8),
            current_move_modes: Cell::new(None),
            next_command_id: 0,
            framing: TcpFraming::new(),
            received_responses: HashMap::new(),
        })
    }

    /// Returns the local UDP port that the robot should send state to.
    pub fn udp_port(&self) -> u16 {
        self.udp_port
    }

    /// Returns the message ID of the most recently received robot state (zero if none yet).
    pub(crate) fn latest_state_message_id(&self) -> u64 {
        self.latest_state_message_id.get()
    }

    /// Returns the motion generator and controller modes (robot-state numbering) of the most
    /// recently received robot state.
    pub(crate) fn latest_state_modes(&self) -> (u8, u8) {
        self.latest_state_modes.get()
    }

    /// Returns the robot mode (robot-state numbering) of the most recently received robot state.
    pub(crate) fn latest_robot_mode(&self) -> u8 {
        self.latest_robot_mode.get()
    }

    /// Records the message ID and modes of the most recently received robot state.
    pub(crate) fn record_state(&self, message_id: u64, robot_mode: u8, motion_generator_mode: u8, controller_mode: u8) {
        self.latest_state_message_id.set(message_id);
        self.latest_robot_mode.set(robot_mode);
        self.latest_state_modes.set((motion_generator_mode, controller_mode));
    }

    /// Returns the motion generator and controller modes (robot-state numbering) of the Move
    /// currently running, or `None` when no motion was started (libfranka's `current_move_*`).
    pub(crate) fn current_move_modes(&self) -> Option<(u8, u8)> {
        self.current_move_modes.get()
    }

    /// Records the modes of the Move being started, or `None` once it has ended.
    pub(crate) fn set_current_move_modes(&self, modes: Option<(u8, u8)>) {
        self.current_move_modes.set(modes);
    }

    /// Send a TCP request and return the assigned command ID.
    pub fn tcp_send_request(&mut self, command: H::Command, payload: &[u8]) -> FrankaResult<u32> {
        let command_id = self.next_command_id;
        self.next_command_id += 1;

        let total_size = (H::SIZE + payload.len()) as u32;

        // Header and payload go out in one write so the request leaves as a single segment.
        let mut message = H::encode(command, command_id, total_size);
        message.extend_from_slice(payload);
        self.tcp
            .write_all(&message)
            .map_err(|e| FrankaError::network_with_source("TCP send request", e))?;

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

        self.tcp_try_read_message()?;
        Ok(self.received_responses.remove(&command_id))
    }

    /// Send data over UDP to the robot's last known state-stream address.
    ///
    /// Requires at least one datagram to have been received first (via
    /// `udp_blocking_receive` or `udp_try_receive`) so the destination address is known;
    /// returns an error otherwise.
    pub fn udp_send(&self, data: &[u8]) -> FrankaResult<()> {
        let peer = self.udp_peer.get().ok_or_else(|| {
            FrankaError::network("UDP send attempted before any state datagram was received")
        })?;
        self.udp
            .send_to(data, peer)
            .map_err(|e| FrankaError::network_with_source("UDP send", e))?;
        Ok(())
    }

    /// Blocking receive from UDP. Returns the number of bytes received and records the
    /// sender's address for subsequent `udp_send` calls. Datagrams from any host other
    /// than the robot are discarded.
    pub fn udp_blocking_receive(&self, buf: &mut [u8]) -> FrankaResult<usize> {
        loop {
            let (received, peer) = self
                .udp
                .recv_from(buf)
                .map_err(|e| FrankaError::network_with_source("UDP receive", e))?;
            if peer.ip() != self.robot_ip {
                continue;
            }
            self.udp_peer.set(Some(peer));
            return Ok(received);
        }
    }

    /// Non-blocking receive from UDP. Returns None if no data available. On success,
    /// records the sender's address for subsequent `udp_send` calls. Datagrams from any
    /// host other than the robot are discarded.
    pub fn udp_try_receive(&self, buf: &mut [u8]) -> FrankaResult<Option<usize>> {
        loop {
            match SockRef::from(&self.udp).recv_from_with_flags(as_uninit(buf), libc::MSG_DONTWAIT) {
                Ok((n, peer)) => match peer.as_socket() {
                    Some(peer) if peer.ip() == self.robot_ip => {
                        self.udp_peer.set(Some(peer));
                        return Ok(Some(n));
                    }
                    _ => continue,
                },
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(None),
                Err(e) => return Err(FrankaError::network_with_source("UDP receive", e)),
            }
        }
    }

    /// Returns an error if the robot has closed the TCP connection (libfranka's
    /// `tcpThrowIfConnectionClosed`): a non-blocking peek that reads zero bytes means the peer
    /// closed it. Pending data is left in place and the socket's blocking mode is not changed.
    pub(crate) fn tcp_throw_if_connection_closed(&self) -> FrankaResult<()> {
        let mut byte = [0u8; 1];
        match tcp_receive_nonblocking(&self.tcp, &mut byte, libc::MSG_PEEK) {
            Ok(0) => Err(FrankaError::network("server closed connection")),
            Ok(_) => Ok(()),
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => Ok(()),
            Err(e) => Err(FrankaError::network_with_source("TCP connection check", e)),
        }
    }

    /// Returns whether the TCP socket reports neither an error nor a hang-up, as libfranka's
    /// `isTcpSocketAlive` (a zero-timeout poll for errors). The socket's pending error is not
    /// cleared.
    pub fn is_tcp_alive(&self) -> bool {
        // Errors and hang-ups are always reported by poll, so no events are requested.
        let mut poll_fd = libc::pollfd { fd: self.tcp.as_raw_fd(), events: 0, revents: 0 };
        // SAFETY: `poll_fd` is a valid pollfd that outlives the call, and the count is 1.
        let ready = unsafe { libc::poll(&mut poll_fd, 1, 0) };
        ready == 0
    }

    // --- Internal helpers ---

    /// Read one complete message from TCP and buffer it.
    fn tcp_read_message(&mut self) -> FrankaResult<()> {
        // Read the header, continuing any header bytes a non-blocking read left behind.
        while self.framing.header_bytes_needed() > 0 {
            let mut header_buf = [0u8; HEADER_BUFFER_SIZE];
            let needed = self.framing.header_bytes_needed();
            let n = self
                .tcp
                .read(&mut header_buf[..needed])
                .map_err(|e| FrankaError::network_with_source("TCP read header", e))?;
            if n == 0 {
                return Err(FrankaError::network("server closed connection"));
            }
            self.framing.push_header_bytes(&header_buf[..n])?;
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
        // Header bytes that arrive split across reads are kept in the framing state, so a
        // would-block in the middle of a header loses nothing.
        while self.framing.header_bytes_needed() > 0 {
            let mut header_buf = [0u8; HEADER_BUFFER_SIZE];
            let needed = self.framing.header_bytes_needed();
            match tcp_receive_nonblocking(&self.tcp, &mut header_buf[..needed], 0) {
                Ok(0) => return Err(FrankaError::network("server closed connection")),
                Ok(n) => self.framing.push_header_bytes(&header_buf[..n])?,
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
                Err(e) => return Err(FrankaError::network_with_source("TCP read header", e)),
            }
        }

        while !self.framing.is_complete() {
            let remaining = self.framing.remaining_bytes();
            let mut chunk = vec![0u8; remaining.min(4096)];
            match tcp_receive_nonblocking(&self.tcp, &mut chunk, 0) {
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

/// Receives from `tcp` into `buf` without blocking, with `flags` added to `MSG_DONTWAIT`; the
/// socket's blocking mode is not changed. Returns a `WouldBlock` error if no data is available.
fn tcp_receive_nonblocking(tcp: &TcpStream, buf: &mut [u8], flags: libc::c_int) -> io::Result<usize> {
    SockRef::from(tcp).recv_with_flags(as_uninit(buf), libc::MSG_DONTWAIT | flags)
}

/// Views an initialized byte buffer as the possibly uninitialized buffer socket2 receives into.
fn as_uninit(buf: &mut [u8]) -> &mut [MaybeUninit<u8>] {
    // SAFETY: `MaybeUninit<u8>` has the layout of `u8`, and the receive calls only write
    // initialized bytes into the buffer.
    unsafe { &mut *(buf as *mut [u8] as *mut [MaybeUninit<u8>]) }
}

impl<H> Drop for Network<H> {
    fn drop(&mut self) {
        let _ = self.tcp.shutdown(std::net::Shutdown::Both);
    }
}
