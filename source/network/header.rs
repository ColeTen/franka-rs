//! TCP message headers of the robot, gripper and vacuum gripper protocols.

use crate::wire::{gripper, robot, vacuum};

/// The fixed-size header that starts every TCP message of one protocol.
///
/// Each header holds a command, a command ID and the total message size (header included); the
/// protocols differ in the width of the command field and therefore in the header size.
pub trait MessageHeader {
    /// Encoded header size in bytes.
    const SIZE: usize;

    /// Type of the header's command field.
    type Command;

    /// Encodes the header of a `size`-byte message (header included).
    fn encode(command: Self::Command, command_id: u32, size: u32) -> Vec<u8>;

    /// Decodes `(command_id, size)` from a header. `bytes` must be at least `SIZE` bytes long.
    fn decode(bytes: &[u8]) -> (u32, u32);
}

impl MessageHeader for robot::CommandHeader {
    const SIZE: usize = robot::CommandHeader::SIZE;
    type Command = u32;

    fn encode(command: u32, command_id: u32, size: u32) -> Vec<u8> {
        robot::CommandHeader { command, command_id, size }.to_bytes().to_vec()
    }

    fn decode(bytes: &[u8]) -> (u32, u32) {
        assert!(bytes.len() >= Self::SIZE, "robot header needs {} bytes", Self::SIZE);
        // SAFETY: `bytes` holds at least `SIZE` bytes, checked above.
        let header = unsafe { robot::CommandHeader::from_bytes(bytes) };
        ({ header.command_id }, { header.size })
    }
}

impl MessageHeader for gripper::CommandHeader {
    const SIZE: usize = gripper::CommandHeader::SIZE;
    type Command = u16;

    fn encode(command: u16, command_id: u32, size: u32) -> Vec<u8> {
        gripper::CommandHeader { command, command_id, size }.to_bytes().to_vec()
    }

    fn decode(bytes: &[u8]) -> (u32, u32) {
        assert!(bytes.len() >= Self::SIZE, "gripper header needs {} bytes", Self::SIZE);
        // SAFETY: `bytes` holds at least `SIZE` bytes, checked above.
        let header = unsafe { gripper::CommandHeader::from_bytes(bytes) };
        ({ header.command_id }, { header.size })
    }
}

impl MessageHeader for vacuum::CommandHeader {
    const SIZE: usize = vacuum::CommandHeader::SIZE;
    type Command = u16;

    fn encode(command: u16, command_id: u32, size: u32) -> Vec<u8> {
        vacuum::CommandHeader { command, command_id, size }.to_bytes().to_vec()
    }

    fn decode(bytes: &[u8]) -> (u32, u32) {
        assert!(bytes.len() >= Self::SIZE, "vacuum gripper header needs {} bytes", Self::SIZE);
        // SAFETY: `bytes` holds at least `SIZE` bytes, checked above.
        let header = unsafe { vacuum::CommandHeader::from_bytes(bytes) };
        ({ header.command_id }, { header.size })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_sizes_fit_the_read_buffer() {
        assert!(<robot::CommandHeader as MessageHeader>::SIZE <= super::super::HEADER_BUFFER_SIZE);
        assert!(<gripper::CommandHeader as MessageHeader>::SIZE <= super::super::HEADER_BUFFER_SIZE);
        assert!(<vacuum::CommandHeader as MessageHeader>::SIZE <= super::super::HEADER_BUFFER_SIZE);
    }

    #[test]
    fn header_sizes_match_the_protocols() {
        assert_eq!(<robot::CommandHeader as MessageHeader>::SIZE, 12);
        assert_eq!(<gripper::CommandHeader as MessageHeader>::SIZE, 10);
        assert_eq!(<vacuum::CommandHeader as MessageHeader>::SIZE, 10);
    }

    #[test]
    fn gripper_header_round_trips() {
        let bytes = gripper::CommandHeader::encode(3, 7, 26);
        let mut expected = Vec::new();
        expected.extend_from_slice(&3u16.to_ne_bytes());
        expected.extend_from_slice(&7u32.to_ne_bytes());
        expected.extend_from_slice(&26u32.to_ne_bytes());
        assert_eq!(bytes, expected);
        assert_eq!(gripper::CommandHeader::decode(&bytes), (7, 26));
    }

    #[test]
    fn robot_header_round_trips() {
        let bytes = robot::CommandHeader::encode(2, 9, 40);
        assert_eq!(bytes.len(), 12);
        assert_eq!(&bytes[..4], &2u32.to_ne_bytes());
        assert_eq!(robot::CommandHeader::decode(&bytes), (9, 40));
    }
}
