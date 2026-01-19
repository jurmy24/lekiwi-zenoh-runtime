// Omniwheel inverse kinematics for LeKiwi 3-wheel base
// Converts body-frame velocities (x, y, theta) to individual wheel velocities.

use std::f32::consts::PI;

/// Wheel configuration for LeKiwi base
pub const WHEEL_RADIUS: f32 = 0.05; // meters
pub const BASE_RADIUS: f32 = 0.125; // meters (distance from center to wheel)

/// Wheel mounting angles (degrees) with -90° offset
/// Left wheel at 240°, Back wheel at 0°, Right wheel at 120° (simple cartesian configuration)
const WHEEL_ANGLES_DEG: [f32; 3] = [240.0 - 90.0, 0.0 - 90.0, 120.0 - 90.0];

/// Motor resolution: 4096 steps per revolution
const STEPS_PER_REVOLUTION: f32 = 4096.0;
const STEPS_PER_DEG: f32 = STEPS_PER_REVOLUTION / 360.0;

/// Maximum raw velocity command (safety limit)
const MAX_RAW: i16 = 3000;

/// Raw wheel velocity commands for the three motors
#[derive(Debug, Clone, Copy, Default)]
pub struct WheelVelocities {
    pub left: i16,  // Motor ID 7
    pub back: i16,  // Motor ID 8
    pub right: i16, // Motor ID 9
}

impl WheelVelocities {
    pub fn new(left: i16, back: i16, right: i16) -> Self {
        Self { left, back, right }
    }

    pub fn zero() -> Self {
        Self::default()
    }

    /// Returns velocities as array [left, back, right]
    pub fn as_array(&self) -> [i16; 3] {
        [self.left, self.back, self.right]
    }
}

/// Convert degrees per second to raw motor ticks
fn degps_to_raw(degps: f32) -> i16 {
    let speed_in_steps = degps * STEPS_PER_DEG;
    let speed_int = speed_in_steps.round() as i32;

    // Clamp to signed 16-bit range
    speed_int.clamp(-0x8000, 0x7FFF) as i16
}

/// Convert body-frame velocities to raw wheel commands
///
/// # Arguments
/// * `x` - Forward velocity in m/s (positive = forward)
/// * `y` - Lateral velocity in m/s (positive = left)
/// * `theta` - Rotational velocity in deg/s (positive = counter-clockwise)
///
/// # Returns
/// Raw wheel velocity commands for each motor
pub fn body_to_wheel_raw(x: f32, y: f32, theta: f32) -> WheelVelocities {
    body_to_wheel_raw_with_params(x, y, theta, WHEEL_RADIUS, BASE_RADIUS, MAX_RAW)
}

/// Convert body-frame velocities to raw wheel commands with custom parameters
pub fn body_to_wheel_raw_with_params(
    x: f32,
    y: f32,
    theta: f32,
    wheel_radius: f32,
    base_radius: f32,
    max_raw: i16,
) -> WheelVelocities {
    // Convert rotational velocity from deg/s to rad/s
    let theta_rad = theta * (PI / 180.0);

    // Body velocity vector [x, y, theta_rad]
    let velocity = [x, y, theta_rad];

    // Build kinematic matrix and compute wheel linear speeds
    // Each row: [cos(angle), sin(angle), base_radius]
    // Maps body velocities to wheel linear speed
    let mut wheel_linear_speeds = [0.0f32; 3];

    for (i, &angle_deg) in WHEEL_ANGLES_DEG.iter().enumerate() {
        let angle_rad = angle_deg * (PI / 180.0);
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();

        // Linear speed = cos(a)*x + sin(a)*y + base_radius*theta_rad
        wheel_linear_speeds[i] =
            cos_a * velocity[0] + sin_a * velocity[1] + base_radius * velocity[2];
    }

    // Convert linear speeds (m/s) to angular speeds (rad/s)
    let wheel_angular_speeds: [f32; 3] = wheel_linear_speeds.map(|linear| linear / wheel_radius);

    // Convert from rad/s to deg/s
    let mut wheel_degps: [f32; 3] = wheel_angular_speeds.map(|radps| radps * (180.0 / PI));

    // Apply scaling if any wheel exceeds max_raw
    let raw_floats: Vec<f32> = wheel_degps
        .iter()
        .map(|&degps| degps.abs() * STEPS_PER_DEG)
        .collect();
    let max_raw_computed = raw_floats.iter().cloned().fold(0.0f32, f32::max);

    if max_raw_computed > max_raw as f32 {
        let scale = max_raw as f32 / max_raw_computed;
        for degps in &mut wheel_degps {
            *degps *= scale;
        }
    }

    // Convert each wheel's angular speed (deg/s) to raw integer
    WheelVelocities {
        left: degps_to_raw(wheel_degps[0]),
        back: degps_to_raw(wheel_degps[1]),
        right: degps_to_raw(wheel_degps[2]),
    }
}

//
///
/// These tests are used to verify the correctness of the kinematics module.
/// They are run using the `cargo test` command.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_velocity() {
        let wheels = body_to_wheel_raw(0.0, 0.0, 0.0);
        assert_eq!(wheels.left, 0);
        assert_eq!(wheels.back, 0);
        assert_eq!(wheels.right, 0);
    }

    #[test]
    fn test_forward_motion() {
        // Moving forward (positive X) with 3-wheel omni configuration:
        // - Back wheel is at -90° (perpendicular to forward), so it doesn't contribute
        // - Left and right wheels spin in opposite directions to move forward
        let wheels = body_to_wheel_raw(0.1, 0.0, 0.0);
        println!(
            "Forward: left={}, back={}, right={}",
            wheels.left, wheels.back, wheels.right
        );

        // Left and right should be opposite signs (symmetric motion)
        assert!(
            wheels.left != 0,
            "Left wheel should be non-zero for forward motion"
        );
        assert!(
            wheels.right != 0,
            "Right wheel should be non-zero for forward motion"
        );
        assert!(
            (wheels.left > 0) != (wheels.right > 0),
            "Left and right wheels should spin opposite directions"
        );
        // Back wheel is perpendicular to forward direction, so it's ~0
        assert!(
            wheels.back.abs() < 10,
            "Back wheel should be near zero for pure forward motion"
        );
    }

    #[test]
    fn test_rotation_only() {
        // Pure rotation should spin all wheels in same direction
        let wheels = body_to_wheel_raw(0.0, 0.0, 45.0);
        println!(
            "Rotation: left={}, back={}, right={}",
            wheels.left, wheels.back, wheels.right
        );
        // All wheels should spin same direction for rotation
        assert!(wheels.left > 0 && wheels.back > 0 && wheels.right > 0);
    }

    #[test]
    fn test_degps_to_raw_limits() {
        // Test that extreme values are clamped
        let max_val = degps_to_raw(100000.0);
        let min_val = degps_to_raw(-100000.0);
        assert_eq!(max_val, 0x7FFF);
        assert_eq!(min_val, -0x8000);
    }

    #[test]
    fn test_slow_velocity_reasonable_output() {
        // Test that a slow velocity (0.02 m/s) produces small raw values
        // This is important for safety - we don't want to accidentally send max speed
        let wheels = body_to_wheel_raw(0.02, 0.0, 0.0);
        println!(
            "Slow forward (0.02 m/s): left={}, back={}, right={}",
            wheels.left, wheels.back, wheels.right
        );

        // At 0.02 m/s, the raw values should be well under 1000 (safety check)
        // MAX_RAW is 3000, so this should be < 10% of max
        assert!(
            wheels.left.abs() < 500,
            "Left wheel raw {} too high for 0.02 m/s",
            wheels.left
        );
        assert!(
            wheels.back.abs() < 500,
            "Back wheel raw {} too high for 0.02 m/s",
            wheels.back
        );
        assert!(
            wheels.right.abs() < 500,
            "Right wheel raw {} too high for 0.02 m/s",
            wheels.right
        );
    }

    #[test]
    fn test_normal_velocity_reasonable_output() {
        // Test that a normal velocity (0.1 m/s) produces reasonable raw values
        let wheels = body_to_wheel_raw(0.1, 0.0, 0.0);
        println!(
            "Normal forward (0.1 m/s): left={}, back={}, right={}",
            wheels.left, wheels.back, wheels.right
        );

        // At 0.1 m/s, the raw values should be under MAX_RAW (3000)
        assert!(wheels.left.abs() < 3000, "Left wheel exceeds MAX_RAW");
        assert!(wheels.back.abs() < 3000, "Back wheel exceeds MAX_RAW");
        assert!(wheels.right.abs() < 3000, "Right wheel exceeds MAX_RAW");
    }

    #[test]
    fn test_max_velocity_clamped() {
        // Test that extreme velocity gets clamped to MAX_RAW
        let wheels = body_to_wheel_raw(10.0, 0.0, 0.0); // Very high speed
        println!(
            "Extreme forward (10 m/s): left={}, back={}, right={}",
            wheels.left, wheels.back, wheels.right
        );

        // Should be clamped to MAX_RAW (3000)
        assert!(wheels.left.abs() <= 3000, "Left wheel not clamped");
        assert!(wheels.back.abs() <= 3000, "Back wheel not clamped");
        assert!(wheels.right.abs() <= 3000, "Right wheel not clamped");
    }
}
