// Timeouts, topics, motor configuration
use std::time::Duration;

// Runtime loop frequency
pub const LOOP_HZ: u64 = 50;

// Command timeout for watchdog
pub const CMD_TIMEOUT: Duration = Duration::from_millis(250);

// Zenoh topics
pub const TOPIC_CMD_BASE: &str = "lekiwi/cmd/base"; // commands
pub const TOPIC_RT_BASE: &str = "lekiwi/rt/base"; // actuation
pub const TOPIC_HEALTH: &str = "lekiwi/state/health"; // health status

// Motor configuration
// Serial port for Feetech motor controller
pub const MOTOR_PORT: &str = "/dev/tty.usbmodem58760432781";

// Enable hardware motor control (set to false for simulation/testing)
pub const MOTOR_ENABLED: bool = true;
