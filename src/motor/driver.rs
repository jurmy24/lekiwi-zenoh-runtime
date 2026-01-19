// High-level motor driver for LeKiwi base
//
// Combines kinematics and Feetech protocol to provide a simple API
// for controlling the omniwheel base.

use tracing::{debug, info, warn};

use super::feetech::{FeetechBus, FeetechError, OperatingMode, Register};
use super::kinematics::{body_to_wheel_raw, WheelVelocities};

/// Motor IDs for the LeKiwi base (as configured in the motors)
pub const MOTOR_ID_LEFT: u8 = 7;
pub const MOTOR_ID_BACK: u8 = 8;
pub const MOTOR_ID_RIGHT: u8 = 9;

/// All base motor IDs
pub const BASE_MOTOR_IDS: [u8; 3] = [MOTOR_ID_LEFT, MOTOR_ID_BACK, MOTOR_ID_RIGHT];

/// High-level motor driver for the LeKiwi omniwheel base
pub struct MotorDriver {
    bus: FeetechBus,
    motor_ids: [u8; 3], // [left, back, right]
}

impl MotorDriver {
    /// Create a new motor driver, connecting to the specified serial port
    pub fn new(port: &str) -> Result<Self, FeetechError> {
        Self::with_motor_ids(port, BASE_MOTOR_IDS)
    }

    /// Create with custom motor IDs
    pub fn with_motor_ids(port: &str, motor_ids: [u8; 3]) -> Result<Self, FeetechError> {
        info!("Opening motor bus on {}", port);
        let bus = FeetechBus::open(port)?;
        Ok(Self { bus, motor_ids })
    }

    /// Initialize the motors for velocity control
    ///
    /// This must be called before sending velocity commands.
    /// It disables torque, sets velocity mode, and re-enables torque.
    pub fn initialize(&mut self) -> Result<(), FeetechError> {
        info!("Initializing motors {:?} for velocity control", self.motor_ids);

        // First, check that all motors are reachable
        for &id in &self.motor_ids {
            match self.bus.ping(id) {
                Ok(true) => debug!("Motor {} responding", id),
                Ok(false) => {
                    warn!("Motor {} not responding to ping", id);
                    return Err(FeetechError::Timeout { id });
                }
                Err(e) => return Err(e),
            }
        }

        // Disable torque on all motors (required before changing operating mode)
        for &id in &self.motor_ids {
            self.bus.disable_torque(id)?;
        }

        // Set velocity mode on all motors
        for &id in &self.motor_ids {
            self.bus.set_operating_mode(id, OperatingMode::Velocity)?;
        }

        // Enable torque on all motors
        for &id in &self.motor_ids {
            self.bus.enable_torque(id)?;
        }

        info!("Motors initialized successfully");
        Ok(())
    }

    /// Send body velocity command to the base
    ///
    /// # Arguments
    /// * `x` - Forward velocity in m/s (positive = forward)
    /// * `y` - Lateral velocity in m/s (positive = left)
    /// * `theta` - Rotational velocity in deg/s (positive = counter-clockwise)
    pub fn set_body_velocity(&mut self, x: f32, y: f32, theta: f32) -> Result<(), FeetechError> {
        let wheels = body_to_wheel_raw(x, y, theta);
        self.set_wheel_velocities(wheels)
    }

    /// Send raw wheel velocities
    pub fn set_wheel_velocities(&mut self, velocities: WheelVelocities) -> Result<(), FeetechError> {
        debug!(
            "Setting wheel velocities: left={}, back={}, right={}",
            velocities.left, velocities.back, velocities.right
        );

        // Use sync_write for efficiency
        let data = [
            (self.motor_ids[0], velocities.left),
            (self.motor_ids[1], velocities.back),
            (self.motor_ids[2], velocities.right),
        ];

        self.bus.sync_write_i16(Register::GoalVelocity, &data)
    }

    /// Stop all motors immediately
    pub fn stop(&mut self) -> Result<(), FeetechError> {
        info!("Stopping all motors");
        self.set_wheel_velocities(WheelVelocities::zero())
    }

    /// Disable torque on all motors (allows free movement)
    pub fn disable_torque(&mut self) -> Result<(), FeetechError> {
        info!("Disabling torque on all motors");
        for &id in &self.motor_ids {
            self.bus.disable_torque(id)?;
        }
        Ok(())
    }

    /// Read current wheel velocities
    pub fn get_wheel_velocities(&mut self) -> Result<WheelVelocities, FeetechError> {
        let left = self.bus.get_velocity(self.motor_ids[0])?;
        let back = self.bus.get_velocity(self.motor_ids[1])?;
        let right = self.bus.get_velocity(self.motor_ids[2])?;

        Ok(WheelVelocities::new(left, back, right))
    }

    /// Check if a motor is reachable
    pub fn ping(&mut self, id: u8) -> Result<bool, FeetechError> {
        self.bus.ping(id)
    }

    /// Get the motor IDs
    pub fn motor_ids(&self) -> [u8; 3] {
        self.motor_ids
    }
}

impl Drop for MotorDriver {
    fn drop(&mut self) {
        // Try to stop motors when driver is dropped (safety measure)
        if let Err(e) = self.stop() {
            warn!("Failed to stop motors on drop: {}", e);
        }
    }
}
