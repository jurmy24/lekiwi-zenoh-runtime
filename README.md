# LeKiwi Zenoh Runtime

Rust-based runtime for controlling the LeKiwi omniwheel mobile robot base via Zenoh messaging.

## Architecture

```
┌─────────────────┐     Zenoh      ┌─────────────────┐      Serial     ┌─────────────┐
│   cmd_publisher │ ──────────────▶│     Runtime     │ ───────────────▶│   Motors    │
│   (keyboard)    │  lekiwi/cmd/   │   (watchdog)    │   /dev/ttyACM0  │  (Feetech)  │
└─────────────────┘     base       └─────────────────┘                 └─────────────┘
                                           │
                                           ▼
                                   lekiwi/rt/base
                                   lekiwi/state/health
```

## Components

- **Runtime** (`src/runtime.rs`): Main 50Hz control loop with watchdog safety
- **Motor Driver** (`src/motor/`): Feetech STS3215 serial protocol and kinematics
- **Keyboard Teleop** (`examples/cmd_publisher.rs`): WASD keyboard control

## Quick Start

### 1. Hardware Setup

Connect the Feetech motor controller to your computer via USB. Find the serial port:

```bash
# Linux
ls /dev/ttyUSB* /dev/ttyACM*

# macOS
ls /dev/tty.usb*
```

Update `src/config.rs` with your port:

```rust
pub const MOTOR_PORT: &str = "/dev/ttyACM0";  // Change to your port
```

### 2. Test Motor Connection

First, verify motors are responding:

```bash
cargo run --example motor_test -- /dev/ttyACM0
```

This will ping each motor, initialize them for velocity control, and run a brief motion test.

### 3. Run the Runtime

Start the main runtime (receives Zenoh commands, sends to motors):

```bash
RUST_LOG=info cargo run
```

### 4. Send Commands

In another terminal, run the keyboard teleop:

```bash
RUST_LOG=info cargo run --example cmd_publisher
```

Controls:
- **W/A/S/D** - Move forward/left/backward/right
- **Z/X** - Rotate counter-clockwise/clockwise
- **R/F** - Increase/decrease speed
- **Q** - Quit

## Configuration

Edit `src/config.rs`:

| Constant | Default | Description |
|----------|---------|-------------|
| `LOOP_HZ` | 50 | Control loop frequency |
| `CMD_TIMEOUT` | 250ms | Watchdog timeout (stops if no command received) |
| `MOTOR_PORT` | `/dev/ttyACM0` | Serial port for motor controller |
| `MOTOR_ENABLED` | true | Set to `false` to run without hardware |

## Motor IDs

The base motors use these IDs (configured in the motors themselves):

| Motor | ID | Position |
|-------|-----|----------|
| Left | 7 | 240° |
| Back | 8 | 0° |
| Right | 9 | 120° |

## Zenoh Topics

| Topic | Direction | Format | Description |
|-------|-----------|--------|-------------|
| `lekiwi/cmd/base` | Subscribe | `{"x_vel": f32, "y_vel": f32, "theta_vel": f32}` | Velocity commands (m/s, deg/s) |
| `lekiwi/rt/base` | Publish | `{"x_vel": f32, "y_vel": f32, "theta_vel": f32}` | Actual actuation sent |
| `lekiwi/state/health` | Publish | `"ok"` or `"cmd_stale"` | Runtime health status |

## Troubleshooting

### Motor not responding

1. Check serial port path is correct
2. Verify motor controller is powered
3. Check motor IDs match (7, 8, 9)
4. Try running motor_test example with verbose logging: `RUST_LOG=debug cargo run --example motor_test`

### Permission denied on serial port

```bash
# Linux: add user to dialout group
sudo usermod -a -G dialout $USER
# Then log out and back in

# Or temporarily:
sudo chmod 666 /dev/ttyACM0
```

### Robot moves in wrong direction

The kinematics assume a specific wheel layout. If directions are inverted, you may need to adjust the motor wiring or modify `src/motor/kinematics.rs`.

## Development

Run tests:
```bash
cargo test
```

Build release:
```bash
cargo build --release
```
