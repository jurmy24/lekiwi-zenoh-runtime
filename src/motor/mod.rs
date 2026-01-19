// Motor control module for LeKiwi omniwheel base
//
// Provides:
// - Omniwheel inverse kinematics (body velocity -> wheel velocities)
// - Feetech STS3215 serial protocol implementation
// - High-level motor driver API

mod driver;
pub mod feetech;
pub mod kinematics;

pub use driver::{MotorDriver, BASE_MOTOR_IDS, MOTOR_ID_BACK, MOTOR_ID_LEFT, MOTOR_ID_RIGHT};
pub use feetech::{FeetechBus, FeetechError};
pub use kinematics::{body_to_wheel_raw, WheelVelocities};
