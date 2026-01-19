// 50 Hz loop with watchdog and motor control
// Note: a watchdog is a safety mechanism that triggers a safe action if something goes wrong
// Eg. without it if teleop crashes and stops sending commands, the runtime will keep running and sending commands to the robot

use std::time::{Duration, Instant};
use tokio::time::interval; // tokio is an async runtime for Rust
use tracing::{error, info, warn}; // better logging (emits events into the void, not stdout - and a subscriber (tracing-subscriber) can listen to them)

// local imports
use crate::config::{CMD_TIMEOUT, LOOP_HZ, MOTOR_ENABLED, MOTOR_PORT, TOPIC_CMD_BASE, TOPIC_HEALTH, TOPIC_RT_BASE};
use crate::messages::{BaseActuation, BaseCommand, RuntimeHealth};
use crate::motor::MotorDriver;

pub struct Runtime {
    latest_cmd: Option<BaseCommand>,
    cmd_received_at: Instant,
    health: RuntimeHealth,
    motor_driver: Option<MotorDriver>,
}

impl Runtime {
    pub fn new() -> Self {
        Self {
            latest_cmd: None,
            cmd_received_at: Instant::now(),
            health: RuntimeHealth::CmdStale, // Start stale until first cmd
            motor_driver: None,
        }
    }

    /// Initialize motor driver
    pub fn init_motors(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if !MOTOR_ENABLED {
            info!("Motor control disabled in config");
            return Ok(());
        }

        info!("Initializing motor driver on {}...", MOTOR_PORT);
        let mut driver = MotorDriver::new(MOTOR_PORT)?;
        driver.initialize()?;
        self.motor_driver = Some(driver);
        info!("Motor driver initialized successfully");
        Ok(())
    }

    /// Process incoming command
    fn on_command(&mut self, cmd: BaseCommand) {
        info!("Received command: {:?}", &cmd);
        self.latest_cmd = Some(cmd);
        self.cmd_received_at = Instant::now();
    }

    /// Compute actuation based on watchdog state
    fn compute_actuation(&mut self) -> BaseActuation {
        let cmd_age = self.cmd_received_at.elapsed();

        if cmd_age > CMD_TIMEOUT {
            // Watchdog triggered - stop the robot
            if self.health != RuntimeHealth::CmdStale {
                warn!("Command stale ({:?} old), stopping robot", cmd_age);
            }
            self.health = RuntimeHealth::CmdStale;
            BaseActuation::default() // Zero velocity
        } else if let Some(ref cmd) = self.latest_cmd {
            self.health = RuntimeHealth::Ok;
            BaseActuation::from(cmd)
        } else {
            // No command ever received
            self.health = RuntimeHealth::CmdStale;
            BaseActuation::default()
        }
    }

    /// Send actuation to motors
    fn send_to_motors(&mut self, actuation: &BaseActuation) {
        if let Some(ref mut driver) = self.motor_driver {
            if let Err(e) = driver.set_body_velocity(
                actuation.x_vel,
                actuation.y_vel,
                actuation.theta_vel,
            ) {
                error!("Failed to send motor command: {}", e);
            }
        }
    }

    /// Stop motors safely
    fn stop_motors(&mut self) {
        if let Some(ref mut driver) = self.motor_driver {
            if let Err(e) = driver.stop() {
                error!("Failed to stop motors: {}", e);
            }
        }
    }
}

pub async fn run() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    info!("Opening Zenoh session...");
    let session = zenoh::open(zenoh::Config::default()).await?;

    info!("Setting up publishers and subscribers...");
    let subscriber = session.declare_subscriber(TOPIC_CMD_BASE).await?;
    let pub_actuation = session.declare_publisher(TOPIC_RT_BASE).await?;
    let pub_health = session.declare_publisher(TOPIC_HEALTH).await?;

    let mut runtime = Runtime::new();

    // Initialize motors (non-fatal if fails - can still run for testing)
    if let Err(e) = runtime.init_motors() {
        warn!("Failed to initialize motors: {}. Running without motor control.", e);
    }

    let mut tick = interval(Duration::from_millis(1000 / LOOP_HZ));

    info!(
        "Runtime started: {}Hz loop, {}ms watchdog timeout",
        LOOP_HZ,
        CMD_TIMEOUT.as_millis()
    );
    info!("Subscribed to: {}", TOPIC_CMD_BASE);
    info!("Publishing to: {}, {}", TOPIC_RT_BASE, TOPIC_HEALTH);
    info!("Motor control: {}", if runtime.motor_driver.is_some() { "ENABLED" } else { "DISABLED" });

    // Setup graceful shutdown
    let shutdown = tokio::signal::ctrl_c();
    tokio::pin!(shutdown);

    loop {
        tokio::select! {
            _ = tick.tick() => {
                // 1. Drain all pending commands (non-blocking), keep latest
                while let Ok(Some(sample)) = subscriber.try_recv() {
                    let payload = sample.payload().to_bytes();
                    match serde_json::from_slice::<BaseCommand>(&payload) {
                        Ok(cmd) => {
                            runtime.on_command(cmd);
                        }
                        Err(e) => {
                            warn!("Failed to parse command: {}", e);
                        }
                    }
                }

                // 2. Compute actuation (includes watchdog logic)
                let actuation = runtime.compute_actuation();

                // 3. Send to motors
                runtime.send_to_motors(&actuation);

                // 4. Publish actuation over Zenoh
                let actuation_json = serde_json::to_string(&actuation)?;
                pub_actuation.put(actuation_json).await?;

                // 5. Publish health
                let health_json = serde_json::to_string(&runtime.health)?;
                pub_health.put(health_json).await?;
            }
            _ = &mut shutdown => {
                info!("Shutdown signal received");
                break;
            }
        }
    }

    // Graceful shutdown: stop motors
    info!("Stopping motors...");
    runtime.stop_motors();
    info!("Runtime shutdown complete");

    Ok(())
}