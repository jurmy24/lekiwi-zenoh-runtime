// Keyboard teleop: WASD move, Z/X rotate, R/F speed, Q quit
use crossterm::{
    event::{self, Event, KeyCode, KeyEvent, KeyEventKind},
    terminal::{disable_raw_mode, enable_raw_mode},
};
use serde_json::json;
use std::time::{Duration, Instant};
use tracing::info;

const SPEEDS: [f64; 3] = [0.05, 0.15, 0.3]; // m/s
const THETA_SPEEDS: [f64; 3] = [15.0, 45.0, 90.0]; // deg/s
const INPUT_TIMEOUT_MS: u64 = 100; // Reset velocities after this much time with no input

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    tracing_subscriber::fmt().with_env_filter("info").init();

    info!("Opening Zenoh session...");
    let session = zenoh::open(zenoh::Config::default()).await?;
    let publisher = session.declare_publisher("lekiwi/cmd/base").await?;

    info!("Controls: WASD=move, Z/X=rotate, R/F=speed, Q=quit");
    info!("Speed: LOW");

    enable_raw_mode()?;
    let result = run_teleop(&publisher).await;
    disable_raw_mode()?;

    result
}

async fn run_teleop(
    publisher: &zenoh::pubsub::Publisher<'_>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut speed_idx: usize = 0;

    // Persistent velocity state
    let mut x_vel = 0.0;
    let mut y_vel = 0.0;
    let mut theta_vel = 0.0;
    let mut last_movement_input = Instant::now();

    loop {
        // Poll for key with 20ms timeout (50Hz effective rate)
        if event::poll(Duration::from_millis(20))? {
            if let Event::Key(KeyEvent { code, kind, .. }) = event::read()? {
                let pressed = kind == KeyEventKind::Press || kind == KeyEventKind::Repeat;

                match code {
                    // Movement - update velocity and refresh timestamp
                    KeyCode::Char('w') if pressed => {
                        x_vel = SPEEDS[speed_idx];
                        last_movement_input = Instant::now();
                    }
                    KeyCode::Char('s') if pressed => {
                        x_vel = -SPEEDS[speed_idx];
                        last_movement_input = Instant::now();
                    }
                    KeyCode::Char('a') if pressed => {
                        y_vel = SPEEDS[speed_idx];
                        last_movement_input = Instant::now();
                    }
                    KeyCode::Char('d') if pressed => {
                        y_vel = -SPEEDS[speed_idx];
                        last_movement_input = Instant::now();
                    }

                    // Rotation
                    KeyCode::Char('z') if pressed => {
                        theta_vel = THETA_SPEEDS[speed_idx];
                        last_movement_input = Instant::now();
                    }
                    KeyCode::Char('x') if pressed => {
                        theta_vel = -THETA_SPEEDS[speed_idx];
                        last_movement_input = Instant::now();
                    }

                    // Speed control
                    KeyCode::Char('r') if pressed => {
                        speed_idx = (speed_idx + 1).min(2);
                        print_speed(speed_idx);
                    }
                    KeyCode::Char('f') if pressed => {
                        speed_idx = speed_idx.saturating_sub(1);
                        print_speed(speed_idx);
                    }

                    // Quit
                    KeyCode::Char('q') | KeyCode::Esc if pressed => break,

                    _ => {}
                }
            }
        }

        // Reset velocities if no movement input for INPUT_TIMEOUT_MS
        if last_movement_input.elapsed() > Duration::from_millis(INPUT_TIMEOUT_MS) {
            x_vel = 0.0;
            y_vel = 0.0;
            theta_vel = 0.0;
        }

        // Always publish at ~50Hz
        let cmd = json!({
            "x_vel": x_vel,
            "y_vel": y_vel,
            "theta_vel": theta_vel
        });
        publisher.put(cmd.to_string()).await?;
    }

    Ok(())
}

fn print_speed(idx: usize) {
    let label = ["LOW", "MED", "HIGH"][idx];
    info!("Speed: {}", label);
}
