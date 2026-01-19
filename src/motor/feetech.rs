// Feetech STS3215 serial protocol implementation
//
// Protocol is similar to Dynamixel Protocol 1.0:
// Packet format: [0xFF, 0xFF, ID, Length, Instruction, Params..., Checksum]

use serialport::{self, SerialPort};
use std::io::{Read, Write};
use std::time::Duration;
use tracing::debug;

/// Default serial configuration for Feetech motors
pub const DEFAULT_BAUDRATE: u32 = 1_000_000;
pub const DEFAULT_TIMEOUT_MS: u64 = 100;

/// Packet header bytes
const HEADER: [u8; 2] = [0xFF, 0xFF];

/// Instruction set
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum Instruction {
    Ping = 0x01,
    Read = 0x02,
    Write = 0x03,
    RegWrite = 0x04,
    Action = 0x05,
    SyncWrite = 0x83,
}

/// Register addresses for STS3215
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum Register {
    // EEPROM area (persists across power cycles)
    ModelNumber = 3, // 2 bytes, read-only
    Id = 5,          // 1 byte
    BaudRate = 6,    // 1 byte

    // RAM area (volatile)
    OperatingMode = 33,   // 1 byte: 0=position, 1=velocity, 2=PWM, 3=step
    TorqueEnable = 40,    // 1 byte: 0=off, 1=on
    GoalPosition = 42,    // 2 bytes
    GoalVelocity = 46,    // 2 bytes (signed, velocity mode)
    Lock = 55,            // 1 byte: 0=unlocked, 1=locked
    PresentPosition = 56, // 2 bytes, read-only
    PresentVelocity = 58, // 2 bytes, read-only (signed)
}

/// Operating modes
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperatingMode {
    Position = 0,
    Velocity = 1,
    Pwm = 2,
    Step = 3,
}

/// Error types for Feetech communication
#[derive(Debug, thiserror::Error)]
pub enum FeetechError {
    #[error("Serial port error: {0}")]
    Serial(#[from] serialport::Error),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Invalid response from motor {id}: {reason}")]
    InvalidResponse { id: u8, reason: String },

    #[error("Checksum mismatch for motor {id}")]
    ChecksumMismatch { id: u8 },

    #[error("Motor {id} returned error status: 0x{status:02X}")]
    MotorError { id: u8, status: u8 },

    #[error("Timeout waiting for response from motor {id}")]
    Timeout { id: u8 },
}

pub type Result<T> = std::result::Result<T, FeetechError>;

/// Feetech motor bus - handles serial communication with motors
pub struct FeetechBus {
    port: Box<dyn SerialPort>,
}

impl FeetechBus {
    /// Open a new connection to the motor bus
    pub fn open(port_name: &str) -> Result<Self> {
        Self::open_with_baudrate(port_name, DEFAULT_BAUDRATE)
    }

    /// Open with custom baudrate
    pub fn open_with_baudrate(port_name: &str, baudrate: u32) -> Result<Self> {
        let port = serialport::new(port_name, baudrate)
            .timeout(Duration::from_millis(DEFAULT_TIMEOUT_MS))
            .open()?;

        Ok(Self { port })
    }

    /// Calculate checksum for a packet (excluding header)
    fn checksum(data: &[u8]) -> u8 {
        let sum: u16 = data.iter().map(|&b| b as u16).sum();
        (!sum & 0xFF) as u8
    }

    /// Build a packet with header and checksum
    fn build_packet(id: u8, instruction: Instruction, params: &[u8]) -> Vec<u8> {
        let length = (params.len() + 2) as u8; // params + instruction + checksum
        let mut packet = Vec::with_capacity(6 + params.len());

        packet.extend_from_slice(&HEADER);
        packet.push(id);
        packet.push(length);
        packet.push(instruction as u8);
        packet.extend_from_slice(params);

        // Checksum over id, length, instruction, params
        let checksum_data = &packet[2..]; // skip header
        packet.push(Self::checksum(checksum_data));

        packet
    }

    /// Send a packet and optionally wait for response
    fn send_packet(&mut self, packet: &[u8]) -> Result<()> {
        self.port.write_all(packet)?;
        self.port.flush()?;
        Ok(())
    }

    /// Read a response packet
    fn read_response(&mut self, expected_id: u8) -> Result<Vec<u8>> {
        let mut header = [0u8; 2];
        self.port.read_exact(&mut header).map_err(|e| {
            if e.kind() == std::io::ErrorKind::TimedOut {
                FeetechError::Timeout { id: expected_id }
            } else {
                FeetechError::Io(e)
            }
        })?;

        if header != HEADER {
            return Err(FeetechError::InvalidResponse {
                id: expected_id,
                reason: format!("Invalid header: {:02X?}", header),
            });
        }

        let mut id_length = [0u8; 2];
        self.port.read_exact(&mut id_length)?;
        let id = id_length[0];
        let length = id_length[1] as usize;

        if id != expected_id {
            return Err(FeetechError::InvalidResponse {
                id: expected_id,
                reason: format!("ID mismatch: expected {}, got {}", expected_id, id),
            });
        }

        // Read remaining bytes (error + params + checksum = length bytes)
        let mut remaining = vec![0u8; length];
        self.port.read_exact(&mut remaining)?;

        // Verify checksum
        let mut checksum_data = vec![id, length as u8];
        checksum_data.extend_from_slice(&remaining[..remaining.len() - 1]);
        let expected_checksum = Self::checksum(&checksum_data);
        let received_checksum = remaining[remaining.len() - 1];

        if expected_checksum != received_checksum {
            return Err(FeetechError::ChecksumMismatch { id });
        }

        // Check error status
        let error_status = remaining[0];
        if error_status != 0 {
            return Err(FeetechError::MotorError {
                id,
                status: error_status,
            });
        }

        // Return parameters (excluding error byte and checksum)
        Ok(remaining[1..remaining.len() - 1].to_vec())
    }

    /// Ping a motor to check if it's connected
    pub fn ping(&mut self, id: u8) -> Result<bool> {
        let packet = Self::build_packet(id, Instruction::Ping, &[]);
        self.send_packet(&packet)?;

        match self.read_response(id) {
            Ok(_) => Ok(true),
            Err(FeetechError::Timeout { .. }) => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// Write a single byte to a register
    pub fn write_u8(&mut self, id: u8, register: Register, value: u8) -> Result<()> {
        let params = [register as u8, value];
        let packet = Self::build_packet(id, Instruction::Write, &params);
        debug!(
            "Write u8 to motor {}: reg={:?}, value={}",
            id, register, value
        );
        self.send_packet(&packet)?;

        // Read status response
        let _ = self.read_response(id)?;
        Ok(())
    }

    /// Write two bytes (little-endian) to a register
    pub fn write_u16(&mut self, id: u8, register: Register, value: u16) -> Result<()> {
        let params = [register as u8, (value & 0xFF) as u8, (value >> 8) as u8];
        let packet = Self::build_packet(id, Instruction::Write, &params);
        debug!(
            "Write u16 to motor {}: reg={:?}, value={}",
            id, register, value
        );
        self.send_packet(&packet)?;

        let _ = self.read_response(id)?;
        Ok(())
    }

    /// Write a signed 16-bit value (for velocity)
    pub fn write_i16(&mut self, id: u8, register: Register, value: i16) -> Result<()> {
        // Feetech uses sign-magnitude encoding for velocity:
        // Bit 15 = direction (1 = negative), Bits 0-14 = magnitude
        let raw = encode_sign_magnitude(value);
        self.write_u16(id, register, raw)
    }

    /// Read a single byte from a register
    pub fn read_u8(&mut self, id: u8, register: Register) -> Result<u8> {
        let params = [register as u8, 1]; // address, length
        let packet = Self::build_packet(id, Instruction::Read, &params);
        self.send_packet(&packet)?;

        let response = self.read_response(id)?;
        if response.is_empty() {
            return Err(FeetechError::InvalidResponse {
                id,
                reason: "Empty response".to_string(),
            });
        }
        Ok(response[0])
    }

    /// Read two bytes (little-endian) from a register
    pub fn read_u16(&mut self, id: u8, register: Register) -> Result<u16> {
        let params = [register as u8, 2]; // address, length
        let packet = Self::build_packet(id, Instruction::Read, &params);
        self.send_packet(&packet)?;

        let response = self.read_response(id)?;
        if response.len() < 2 {
            return Err(FeetechError::InvalidResponse {
                id,
                reason: format!("Expected 2 bytes, got {}", response.len()),
            });
        }
        Ok(u16::from_le_bytes([response[0], response[1]]))
    }

    /// Sync write: write same register to multiple motors efficiently
    /// data: [(id, value), ...]
    pub fn sync_write_u16(&mut self, register: Register, data: &[(u8, u16)]) -> Result<()> {
        if data.is_empty() {
            return Ok(());
        }

        // Sync write format:
        // [start_addr, data_length, id1, data1_lo, data1_hi, id2, data2_lo, data2_hi, ...]
        let data_length: u8 = 2; // 2 bytes per motor
        let mut params = vec![register as u8, data_length];

        for &(id, value) in data {
            params.push(id);
            params.push((value & 0xFF) as u8);
            params.push((value >> 8) as u8);
        }

        // Broadcast ID for sync write
        let packet = Self::build_packet(0xFE, Instruction::SyncWrite, &params);
        debug!("Sync write to {} motors: reg={:?}", data.len(), register);
        self.send_packet(&packet)?;

        // Sync write has no response
        Ok(())
    }

    /// Sync write signed 16-bit values (for velocities)
    pub fn sync_write_i16(&mut self, register: Register, data: &[(u8, i16)]) -> Result<()> {
        let encoded: Vec<(u8, u16)> = data
            .iter()
            .map(|&(id, val)| (id, encode_sign_magnitude(val)))
            .collect();
        self.sync_write_u16(register, &encoded)
    }

    // === High-level convenience methods ===

    /// Enable torque on a motor
    pub fn enable_torque(&mut self, id: u8) -> Result<()> {
        self.write_u8(id, Register::TorqueEnable, 1)?;
        self.write_u8(id, Register::Lock, 1)
    }

    /// Disable torque on a motor
    pub fn disable_torque(&mut self, id: u8) -> Result<()> {
        self.write_u8(id, Register::TorqueEnable, 0)?;
        self.write_u8(id, Register::Lock, 0)
    }

    /// Set operating mode (must disable torque first)
    pub fn set_operating_mode(&mut self, id: u8, mode: OperatingMode) -> Result<()> {
        self.write_u8(id, Register::OperatingMode, mode as u8)
    }

    /// Set goal velocity for a motor (must be in velocity mode)
    pub fn set_velocity(&mut self, id: u8, velocity: i16) -> Result<()> {
        self.write_i16(id, Register::GoalVelocity, velocity)
    }

    /// Read present velocity from a motor
    pub fn get_velocity(&mut self, id: u8) -> Result<i16> {
        let raw = self.read_u16(id, Register::PresentVelocity)?;
        Ok(decode_sign_magnitude(raw))
    }
}

/// Encode a signed value to sign-magnitude format
/// Bit 15 = sign (1 = negative), Bits 0-14 = magnitude
fn encode_sign_magnitude(value: i16) -> u16 {
    if value >= 0 {
        value as u16
    } else {
        (0x8000 | (-value as u16)) & 0xFFFF
    }
}

/// Decode sign-magnitude format to signed value
fn decode_sign_magnitude(raw: u16) -> i16 {
    let magnitude = (raw & 0x7FFF) as i16;
    if raw & 0x8000 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_checksum() {
        // Example: ID=1, Length=4, Instruction=WRITE, Addr=30, Data=0, 2
        let data = [1u8, 4, 0x03, 30, 0, 2];
        let checksum = FeetechBus::checksum(&data);
        // ~(1+4+3+30+0+2) = ~40 = 215
        assert_eq!(checksum, 215);
    }

    #[test]
    fn test_sign_magnitude_encoding() {
        assert_eq!(encode_sign_magnitude(0), 0);
        assert_eq!(encode_sign_magnitude(100), 100);
        assert_eq!(encode_sign_magnitude(-100), 0x8064); // 0x8000 | 100
        assert_eq!(encode_sign_magnitude(-1), 0x8001);

        assert_eq!(decode_sign_magnitude(0), 0);
        assert_eq!(decode_sign_magnitude(100), 100);
        assert_eq!(decode_sign_magnitude(0x8064), -100);
        assert_eq!(decode_sign_magnitude(0x8001), -1);
    }

    #[test]
    fn test_build_packet() {
        let packet = FeetechBus::build_packet(1, Instruction::Ping, &[]);
        // Header (2) + ID (1) + Length (1) + Instruction (1) + Checksum (1) = 6 bytes
        assert_eq!(packet.len(), 6);
        assert_eq!(packet[0], 0xFF);
        assert_eq!(packet[1], 0xFF);
        assert_eq!(packet[2], 1); // ID
        assert_eq!(packet[3], 2); // Length (instruction + checksum)
        assert_eq!(packet[4], 0x01); // PING instruction
    }
}
