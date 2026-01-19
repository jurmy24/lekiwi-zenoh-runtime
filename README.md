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

## Deployment Modes

This runtime supports two deployment modes:

### Local Mode (Single Machine)
Run both runtime and keyboard control on the same machine. Uses Zenoh multicast discovery - no configuration needed.

### Network Mode (Distributed)
Run the runtime on a Raspberry Pi (connected to motors) and control from a separate computer over WiFi.

```
┌─────────────────────────────────────────────────────────────────────┐
│                         WiFi Network                                 │
│                                                                      │
│   ┌─────────────────────────┐      TCP/7447      ┌────────────────┐ │
│   │   Raspberry Pi (Host)   │◀─────────────────▶│  Your Computer │ │
│   │   runtime --listen      │                    │  cmd_publisher │ │
│   │   + Motors (USB serial) │                    │  --connect IP  │ │
│   └─────────────────────────┘                    └────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

---

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

---

## Network Mode (Raspberry Pi + Computer)

For distributed deployment over WiFi (e.g., hotspot from your phone or computer):

### 1. On the Raspberry Pi (Host)

First, find the Pi's IP address:

```bash
hostname -I
# Example output: 192.168.43.42
```

Update the serial port in `src/config.rs` if needed (typically `/dev/ttyUSB0` or `/dev/ttyACM0` on Linux).

Start the runtime in listen mode:

```bash
RUST_LOG=info cargo run -- --listen
# Or with custom port:
RUST_LOG=info cargo run -- --listen --port 7447
```

The runtime will print:
```
Network mode: listening on tcp/0.0.0.0:7447
```

### 2. On Your Computer (Client)

Connect to the same WiFi network as the Raspberry Pi.

Run the keyboard teleop with the Pi's IP address:

```bash
RUST_LOG=info cargo run --example cmd_publisher -- --connect 192.168.43.42
# Or with custom port:
RUST_LOG=info cargo run --example cmd_publisher -- --connect 192.168.43.42 --port 7447
```

The client will print:
```
Network mode: connecting to tcp/192.168.43.42:7447
```

### Cross-Compiling for Raspberry Pi

For Raspberry Pi 4/5 (64-bit):

```bash
# Add target
rustup target add aarch64-unknown-linux-gnu

# Install linker (macOS with Homebrew)
brew install messense/macos-cross-toolchains/aarch64-unknown-linux-gnu

# Build
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-unknown-linux-gnu-gcc \
cargo build --release --target aarch64-unknown-linux-gnu

# Copy binary to Pi
scp target/aarch64-unknown-linux-gnu/release/lekiwi-zenoh-runtime pi@192.168.43.42:~
```

For Raspberry Pi 3 or older (32-bit):

```bash
rustup target add armv7-unknown-linux-gnueabihf
```

### Firewall Notes

Ensure port 7447 (TCP) is open on the Raspberry Pi:

```bash
# Check if firewall is active
sudo ufw status

# Allow port if needed
sudo ufw allow 7447/tcp
```

---

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

### Network mode: client can't connect

1. Verify both devices are on the same WiFi network
2. Check the Pi's IP address: `hostname -I`
3. Test connectivity: `ping 192.168.x.x`
4. Ensure port 7447 is open: `sudo ufw allow 7447/tcp`
5. Check the runtime is running with `--listen` flag

### Network mode: connection drops or high latency

1. Use a stable WiFi connection (5GHz preferred over 2.4GHz)
2. Move closer to the access point
3. If using a phone hotspot, ensure it's not going to sleep

## Development

Run tests:
```bash
cargo test
```

Build release:
```bash
cargo build --release
```
