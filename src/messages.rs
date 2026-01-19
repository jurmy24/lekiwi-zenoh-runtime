// Define message types for the runtime

use serde::{Deserialize, Serialize};

// Command from teleop/scripts -> runtime
// derive macro auto-implements print/debug, cloning, and (de)serialization for the following struct/enum
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BaseCommand {
    pub x_vel: f32,
    pub y_vel: f32,
    pub theta_vel: f32,
}

// Actuation output from runtime -> lekiwi-hw
// Has default values because we don't always have an actuation to send
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct BaseActuation {
    pub x_vel: f32,
    pub y_vel: f32,
    pub theta_vel: f32,
}

// Defines how to create a BaseActuation from a borrowed BaseCommand
// from copies the three velocity fields and returns a new BaseActuation
impl From<&BaseCommand> for BaseActuation {
    fn from(cmd: &BaseCommand) -> Self {
        Self {
            x_vel: cmd.x_vel,
            y_vel: cmd.y_vel,
            theta_vel: cmd.theta_vel,
        }
    }
}

/// Health status published by runtime
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum RuntimeHealth {
    Ok,
    CmdStale,
}
