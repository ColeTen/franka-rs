use crate::errors::{FrankaError, FrankaResult};
use crate::wire::robot::CommandHeader;

/// Handles TCP message framing: assembles partial reads into complete messages.
///
/// The protocol uses a fixed-size header (CommandHeader: 12 bytes) followed by
/// a variable-length payload. The header's `size` field gives the total message
/// size (including the header itself).
pub(super) struct TcpFraming {
    state: FramingState,
}

enum FramingState {
    /// Waiting for a new message header.
    Idle,
    /// Header received, accumulating payload bytes.
    Reading {
        command_id: u32,
        buffer: Vec<u8>,
        expected_size: usize,
    },
}

impl TcpFraming {
    pub fn new() -> Self {
        Self {
            state: FramingState::Idle,
        }
    }

    /// Returns true if we have already parsed a header and are reading payload.
    pub fn has_pending_header(&self) -> bool {
        matches!(self.state, FramingState::Reading { .. })
    }

    /// Parse a header and begin accumulating the message body.
    pub fn set_header(&mut self, header_bytes: &[u8; CommandHeader::SIZE]) -> FrankaResult<()> {
        let header = unsafe { CommandHeader::from_bytes(header_bytes) };
        let size = { header.size } as usize;
        let command_id = { header.command_id };

        if size < CommandHeader::SIZE {
            return Err(FrankaError::Protocol {
                message: format!(
                    "TCP message size ({size}) is smaller than header ({})",
                    CommandHeader::SIZE
                ),
            });
        }

        let mut buffer = Vec::with_capacity(size);
        buffer.extend_from_slice(header_bytes);

        self.state = FramingState::Reading {
            command_id,
            buffer,
            expected_size: size,
        };

        Ok(())
    }

    /// Append received bytes to the current message being assembled.
    pub fn push_bytes(&mut self, data: &[u8]) {
        if let FramingState::Reading { buffer, .. } = &mut self.state {
            buffer.extend_from_slice(data);
        }
    }

    /// Returns the number of bytes still needed to complete the current message.
    pub fn remaining_bytes(&self) -> usize {
        match &self.state {
            FramingState::Idle => 0,
            FramingState::Reading {
                buffer,
                expected_size,
                ..
            } => expected_size.saturating_sub(buffer.len()),
        }
    }

    /// Returns true if the current message is fully assembled.
    pub fn is_complete(&self) -> bool {
        match &self.state {
            FramingState::Idle => false,
            FramingState::Reading {
                buffer,
                expected_size,
                ..
            } => buffer.len() >= *expected_size,
        }
    }

    /// Extract the completed message, returning (command_id, full_message_bytes).
    /// Resets state to Idle.
    ///
    /// # Panics
    /// Panics if called when the message is not yet complete.
    pub fn take_message(&mut self) -> (u32, Vec<u8>) {
        let old_state = std::mem::replace(&mut self.state, FramingState::Idle);
        match old_state {
            FramingState::Reading {
                command_id, buffer, ..
            } => (command_id, buffer),
            FramingState::Idle => panic!("take_message called with no pending message"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn framing_basic_message() {
        let mut framing = TcpFraming::new();
        assert!(!framing.has_pending_header());

        // Create a header: command=1, command_id=42, size=20 (12 header + 8 payload)
        let header = CommandHeader {
            command: 1,
            command_id: 42,
            size: 20,
        };
        let header_bytes = header.to_bytes();

        framing.set_header(&header_bytes).unwrap();
        assert!(framing.has_pending_header());
        assert!(!framing.is_complete());
        assert_eq!(framing.remaining_bytes(), 8);

        // Push 4 bytes
        framing.push_bytes(&[0xAA; 4]);
        assert_eq!(framing.remaining_bytes(), 4);
        assert!(!framing.is_complete());

        // Push remaining 4 bytes
        framing.push_bytes(&[0xBB; 4]);
        assert_eq!(framing.remaining_bytes(), 0);
        assert!(framing.is_complete());

        let (cmd_id, msg) = framing.take_message();
        assert_eq!(cmd_id, 42);
        assert_eq!(msg.len(), 20);
        assert!(!framing.has_pending_header());
    }

    #[test]
    fn framing_header_only_message() {
        let mut framing = TcpFraming::new();

        // A message with size == header size (no payload)
        let header = CommandHeader {
            command: 0,
            command_id: 7,
            size: 12,
        };
        let header_bytes = header.to_bytes();

        framing.set_header(&header_bytes).unwrap();
        assert!(framing.is_complete());

        let (cmd_id, msg) = framing.take_message();
        assert_eq!(cmd_id, 7);
        assert_eq!(msg.len(), 12);
    }

    #[test]
    fn framing_rejects_too_small_size() {
        let mut framing = TcpFraming::new();

        let header = CommandHeader {
            command: 0,
            command_id: 0,
            size: 4, // Less than header size (12)
        };
        let header_bytes = header.to_bytes();

        let result = framing.set_header(&header_bytes);
        assert!(result.is_err());
    }
}
