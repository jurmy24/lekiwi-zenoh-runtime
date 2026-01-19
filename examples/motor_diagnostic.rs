// Motor diagnostic: READ-ONLY test to verify motor connection
//
// This tool does NOT write anything to the motors - it's completely safe.
// Use this first before running motor_test.
//
// Usage: cargo run --example motor_diagnostic -- [port]
// Example: cargo run --example motor_diagnostic -- /dev/tty.usbmodem58760432781

use lekiwi_zenoh_runtime::motor::feetech::{FeetechBus, Register};
use std::io::{self, Write};

const MOTOR_IDS: [u8; 3] = [7, 8, 9];
const MOTOR_NAMES: [&str; 3] = ["Left", "Back", "Right"];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Setup logging
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::from_default_env()
                .add_directive("debug".parse().unwrap()),
        )
        .init();

    // Get port from args or use default
    let port = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "/dev/tty.usbmodem58760432781".to_string());

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║           LeKiwi Motor Diagnostic (READ-ONLY)                ║");
    println!("╠══════════════════════════════════════════════════════════════╣");
    println!("║  This tool only READS from motors - no writes, no movement   ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();
    println!("Serial port: {}", port);
    println!("Expected motor IDs: {:?}", MOTOR_IDS);
    println!();

    // Try to open serial port
    println!("Step 1: Opening serial port...");
    let mut bus = match FeetechBus::open(&port) {
        Ok(bus) => {
            println!("  ✓ Serial port opened successfully");
            bus
        }
        Err(e) => {
            println!("  ✗ Failed to open serial port: {}", e);
            println!();
            println!("Troubleshooting:");
            println!("  - Check the port path is correct");
            println!("  - Verify the USB cable is connected");
            println!("  - On macOS, check System Preferences > Security for permission");
            return Err(e.into());
        }
    };
    println!();

    // Ping each motor
    println!("Step 2: Pinging motors...");
    let mut all_found = true;
    for (i, &id) in MOTOR_IDS.iter().enumerate() {
        print!("  Motor {} (ID {}): ", MOTOR_NAMES[i], id);
        io::stdout().flush()?;
        
        match bus.ping(id) {
            Ok(true) => println!("✓ RESPONDING"),
            Ok(false) => {
                println!("✗ NO RESPONSE");
                all_found = false;
            }
            Err(e) => {
                println!("✗ ERROR: {}", e);
                all_found = false;
            }
        }
    }
    println!();

    if !all_found {
        println!("⚠ WARNING: Not all motors responded!");
        println!("  - Check motor power supply");
        println!("  - Verify motor IDs are 7, 8, 9");
        println!("  - Check wiring connections");
        println!();
        print!("Continue reading available motors? [y/N]: ");
        io::stdout().flush()?;
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        if !input.trim().eq_ignore_ascii_case("y") {
            println!("Aborted.");
            return Ok(());
        }
        println!();
    }

    // Read registers from each motor
    println!("Step 3: Reading motor registers...");
    println!();
    
    for (i, &id) in MOTOR_IDS.iter().enumerate() {
        println!("  === Motor {} (ID {}) ===", MOTOR_NAMES[i], id);
        
        // Try to read operating mode
        match bus.read_u8(id, Register::OperatingMode) {
            Ok(mode) => {
                let mode_str = match mode {
                    0 => "Position",
                    1 => "Velocity",
                    2 => "PWM",
                    3 => "Step",
                    _ => "Unknown",
                };
                println!("    Operating Mode: {} ({})", mode, mode_str);
            }
            Err(e) => println!("    Operating Mode: ERROR - {}", e),
        }

        // Torque enable
        match bus.read_u8(id, Register::TorqueEnable) {
            Ok(val) => {
                let status = if val == 1 { "ENABLED" } else { "disabled" };
                println!("    Torque Enable:  {} ({})", val, status);
            }
            Err(e) => println!("    Torque Enable:  ERROR - {}", e),
        }

        // Lock
        match bus.read_u8(id, Register::Lock) {
            Ok(val) => {
                let status = if val == 1 { "LOCKED" } else { "unlocked" };
                println!("    Lock:           {} ({})", val, status);
            }
            Err(e) => println!("    Lock:           ERROR - {}", e),
        }

        // Present velocity
        match bus.get_velocity(id) {
            Ok(vel) => {
                println!("    Present Velocity: {} (raw)", vel);
            }
            Err(e) => println!("    Present Velocity: ERROR - {}", e),
        }

        // Present position
        match bus.read_u16(id, Register::PresentPosition) {
            Ok(pos) => {
                let degrees = (pos as f32) * 360.0 / 4096.0;
                println!("    Present Position: {} ({:.1}°)", pos, degrees);
            }
            Err(e) => println!("    Present Position: ERROR - {}", e),
        }

        println!();
    }

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║                    Diagnostic Complete                       ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();
    println!("If all motors responded and show reasonable values:");
    println!("  1. Motors should show Operating Mode = 1 (Velocity) if previously configured");
    println!("  2. If Operating Mode = 0 (Position), the motor_test will configure them");
    println!("  3. Present Velocity should be 0 or near 0 when wheels are stationary");
    println!();
    println!("Next step: Run 'cargo run --example motor_test' with wheels OFF THE GROUND");

    Ok(())
}
