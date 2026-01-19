// 50 Hz loop with watchdog
// Note: a watchdog is a safety mechanism that triggers a safe action if something goes wrong
// Eg. without it if teleop crashes and stops sending commands, the runtime will keep running and sending commands to the robot

use std::time::{Duration, Instant};
use tokio::time::interval; // tokio is an async runtime for Rust
use tracing::{info, warn}; // better logging (emits events into the void, not stdout - and a subscriber (tracing-subscriber) can listen to them)

// local imports
use crate::config::{CMD_TIMEOUT, LOOP_HZ, TOPIC_CMD_BASE, TOPIC_HEALTH, TOPIC_RT_BASE};
use crate::messages::{BaseActuation, BaseCommand, RuntimeHealth};

pub struct Runtime {
    latest_cmd: Option<BaseCommand>,
    cmd_received_at: Instant,
    health: RuntimeHealth,
}

impl Runtime {
    pub fn new() -> Self {
        Self {
            latest_cmd: None,
            cmd_received_at: Instant::now(),
            health: RuntimeHealth::CmdStale, // Start stale until first cmd
        }
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
}

pub async fn run() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    info!("Opening Zenoh session...");
    let session = zenoh::open(zenoh::Config::default()).await?;

    info!("Setting up publishers and subscribers...");
    let subscriber = session.declare_subscriber(TOPIC_CMD_BASE).await?;
    let pub_actuation = session.declare_publisher(TOPIC_RT_BASE).await?;
    let pub_health = session.declare_publisher(TOPIC_HEALTH).await?;

    let mut runtime = Runtime::new();
    let mut tick = interval(Duration::from_millis(1000 / LOOP_HZ));

    info!(
        "Runtime started: {}Hz loop, {}ms watchdog timeout",
        LOOP_HZ,
        CMD_TIMEOUT.as_millis()
    );
    info!("Subscribed to: {}", TOPIC_CMD_BASE);
    info!("Publishing to: {}, {}", TOPIC_RT_BASE, TOPIC_HEALTH);

    loop {
        tick.tick().await;

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

        // 3. Publish actuation
        let actuation_json = serde_json::to_string(&actuation)?;
        pub_actuation.put(actuation_json).await?;

        // 4. Publish health
        let health_json = serde_json::to_string(&runtime.health)?;
        pub_health.put(health_json).await?;
    }
}