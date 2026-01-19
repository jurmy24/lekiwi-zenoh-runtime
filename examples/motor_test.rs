// Motor test: Careful, step-by-step test for motor control
//
// IMPORTANT: Run motor_diagnostic FIRST to verify read-only communication.
//
// Usage: cargo run --example motor_test -- [port]
// Example: cargo run --example motor_test -- /dev/tty.usbmodem58760432781
//
// Safety features:
// - Explicit confirmation before any writes
// - Starts with zero velocity
// - Very slow test speeds
// - Easy abort with Ctrl+C

use lekiwi_zenoh_runtime::motor::feetech::{FeetechBus, OperatingMode};
use lekiwi_zenoh_runtime::motor::kinematics::{body_to_wheel_raw, WheelVelocities};
use std::io::{self, Write};
use std::thread::sleep;
use std::time::Duration;

const MOTOR_IDS: [u8; 3] = [7, 8, 9];
const MOTOR_NAMES: [&str; 3] = ["Left", "Back", "Right"];

fn confirm(prompt: &str) -> bool {
    print!("{} [y/N]: ", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().eq_ignore_ascii_case("y")
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Setup logging
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive("info".parse().unwrap()),
        )
        .init();

    // Get port from args or use default
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/tty.usbmodem58760432781".to_string());

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║              LeKiwi Motor Test (WITH WRITES)                 ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!("║  ⚠  This tool WILL write to motors and cause movement!       ║");
    println!("║  ⚠  Make sure wheels are OFF THE GROUND before proceeding!   ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();
    println!("Serial port: {}", port);
    println!();

    if !confirm("Have you run motor_diagnostic first and verified all motors respond?") {
        println!(
            "Please run: cargo run --example motor_diagnostic -- {}",
            port
        );
        return Ok(());
    }

    if !confirm("Are the robot's wheels OFF THE GROUND (robot elevated/on blocks)?") {
        println!("Please elevate the robot so wheels can spin freely without moving the robot.");
        return Ok(());
    }

    println!();
    println!("Opening serial port...");
    let mut bus = FeetechBus::open(&port)?;
    println!("✓ Connected");
    println!();

    // ========== STEP 1: Verify communication (read-only) ==========
    println!("Step 1: Verifying motor communication (read-only)...");
    for (i, &id) in MOTOR_IDS.iter().enumerate() {
        match bus.ping(id) {
            Ok(true) => println!("  ✓ Motor {} (ID {}) responding", MOTOR_NAMES[i], id),
            Ok(false) => {
                println!(
                    "  ✗ Motor {} (ID {}) NOT responding - aborting",
                    MOTOR_NAMES[i], id
                );
                return Ok(());
            }
            Err(e) => {
                println!(
                    "  ✗ Motor {} (ID {}) error: {} - aborting",
                    MOTOR_NAMES[i], id, e
                );
                return Ok(());
            }
        }
    }
    println!();

    // ========== STEP 2: Configure for velocity mode ==========
    println!("Step 2: Configuring motors for velocity mode...");
    println!("  This will: disable torque → set velocity mode → enable torque");
    println!();

    if !confirm("Proceed with motor configuration?") {
        println!("Aborted.");
        return Ok(());
    }

    // Disable torque first
    println!("  Disabling torque...");
    for &id in &MOTOR_IDS {
        bus.disable_torque(id)?;
    }
    println!("  ✓ Torque disabled");

    // Set velocity mode
    println!("  Setting velocity mode...");
    for &id in &MOTOR_IDS {
        bus.set_operating_mode(id, OperatingMode::Velocity)?;
    }
    println!("  ✓ Velocity mode set");

    // Enable torque
    println!("  Enabling torque...");
    for &id in &MOTOR_IDS {
        bus.enable_torque(id)?;
    }
    println!("  ✓ Torque enabled");
    println!();

    // ========== STEP 3: Test zero velocity ==========
    println!("Step 3: Sending ZERO velocity to all motors...");
    println!("  This should NOT cause any movement.");
    println!();

    if !confirm("Send zero velocity command?") {
        stop_motors(&mut bus)?;
        return Ok(());
    }

    let zero = WheelVelocities::zero();
    send_wheel_velocities(&mut bus, &zero)?;
    println!("  ✓ Zero velocity sent");
    sleep(Duration::from_millis(500));

    // Verify velocities are zero
    println!("  Reading back velocities...");
    for (i, &id) in MOTOR_IDS.iter().enumerate() {
        let vel = bus.get_velocity(id)?;
        println!(
            "    Motor {} velocity: {} (should be ~0)",
            MOTOR_NAMES[i], vel
        );
    }
    println!();

    // ========== STEP 4: Very slow motion test ==========
    println!("Step 4: Very slow motion test");
    println!("  Speed: 0.02 m/s (very slow walking pace)");
    println!("  Duration: 0.3 seconds per direction");
    println!();
    println!("  ⚠  WATCH THE WHEELS - they should spin slowly!");
    println!("  ⚠  Press Ctrl+C at any time to abort!");
    println!();

    if !confirm("Proceed with motion test?") {
        stop_motors(&mut bus)?;
        return Ok(());
    }

    // Very conservative test parameters
    let test_velocity = 0.02; // m/s - very slow
    let test_duration = Duration::from_millis(300);
    let pause_duration = Duration::from_millis(500);

    // Test sequence
    let tests = [
        ("Forward", test_velocity, 0.0, 0.0),
        ("Backward", -test_velocity, 0.0, 0.0),
        ("Left", 0.0, test_velocity, 0.0),
        ("Right", 0.0, -test_velocity, 0.0),
        ("Rotate CCW", 0.0, 0.0, 15.0), // 15 deg/s - very slow rotation
        ("Rotate CW", 0.0, 0.0, -15.0),
    ];

    for (name, x, y, theta) in tests {
        println!("  Testing: {}...", name);

        let wheels = body_to_wheel_raw(x, y, theta);
        println!(
            "    Wheel commands: left={}, back={}, right={}",
            wheels.left, wheels.back, wheels.right
        );

        send_wheel_velocities(&mut bus, &wheels)?;
        sleep(test_duration);

        // Stop between tests
        send_wheel_velocities(&mut bus, &WheelVelocities::zero())?;
        sleep(pause_duration);
    }

    // ========== FINAL: Stop and cleanup ==========
    println!();
    println!("Step 5: Stopping motors...");
    stop_motors(&mut bus)?;
    println!("  ✓ Motors stopped");

    println!();
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║                    Test Complete!                            ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();
    println!("If the wheels moved as expected, the motor control is working correctly.");
    println!("You can now try the full runtime with: cargo run");

    Ok(())
}

fn send_wheel_velocities(
    bus: &mut FeetechBus,
    vel: &WheelVelocities,
) -> Result<(), Box<dyn std::error::Error>> {
    // Use sync_write for efficiency
    use lekiwi_zenoh_runtime::motor::feetech::Register;

    // Encode sign-magnitude and send via sync_write
    let data = [
        (MOTOR_IDS[0], vel.left),
        (MOTOR_IDS[1], vel.back),
        (MOTOR_IDS[2], vel.right),
    ];

    bus.sync_write_i16(Register::GoalVelocity, &data)?;
    Ok(())
}

fn stop_motors(bus: &mut FeetechBus) -> Result<(), Box<dyn std::error::Error>> {
    // Send zero velocity
    let zero = WheelVelocities::zero();
    send_wheel_velocities(bus, &zero)?;

    // Disable torque for safety
    for &id in &MOTOR_IDS {
        let _ = bus.disable_torque(id); // Ignore errors on cleanup
    }

    Ok(())
}
