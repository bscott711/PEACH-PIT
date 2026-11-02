# PEACH PIT - Robotic Cell Dropper

![PEACH PIT Header](HEADER.png)

An ESP32-based embedded control system for automated laboratory cell handling with precision motion control.

## Overview

PEACH_PIT is a multi-axis robotic control system designed for pick-and-drop operations in laboratory environments. Built on the ESP32 platform with FreeRTOS, it provides deterministic real-time control of stepper motors, servos, and linear actuators.

**Current Branch Status**: v3.0 - Fully refactored to an Active Object architecture with lock-free queues, dynamic NVS limit mapping for autonomous sequences, and adjustable UI speeds.

## Features

- **Multi-task FreeRTOS Architecture**: Concurrent task management for motor control, encoder reading, LCD display, servo positioning, and actuator control.
- **Dynamic Sequence Engine**: Interruptible step-based autonomous operation with E-STOP support and dynamic NVS limit lookups.
- **Precision Motion Control**:
  - Z-Axis Stepper motor with TMC2209 driver and optional optical endstops.
  - Servo motor (Arm) with NVS-persisted 3-point calibration targets (Clear, Buffer, Tip).
  - Linear actuator with DRV8871 H-Bridge driver, position tracking, and adjustable PWM speeds.
- **Real-time Feedback**: Adafruit Seesaw I2C Quadrature encoder integration for closed-loop position control and UI manipulation.
- **User Interface**: SPI OLED LCD display (U8g2) for status monitoring, limit setting, and speed adjustments.
- **Safety Features**:
  - Emergency stop (E-STOP) with immediate sequence interruption.
  - Interlocks preventing Z-axis collisions while the Arm is swung out.
  - Hardware optical endstop overrides.
- **State Persistence**: Non-volatile storage (NVS) for physical limits (Z, Actuator, and Arm) and custom sequence speeds (Jog/Go for all axes).

## Hardware Requirements

- **Microcontroller**: ESP32 DevKit
- **Motor Drivers**:
  - TMC2209 stepper motor driver (UART configurable)
  - DRV8871 H-Bridge (Linear Actuator)
- **Display**: SPI OLED LCD
- **Encoder**: Adafruit I2C Quadrature rotary encoder (4 encoders)
- **Sensors**: Optical Endstops for Z-axis

## Installation

### Prerequisites

- [PlatformIO](https://platformio.org/install) or VS Code with PlatformIO extension
- Python 3.x

### Setup

1. Clone the repository:

```bash
git clone https://gitlab.com/ericjohnson0987/robotic-cell-dropper.git
cd robotic-cell-dropper
```

1. Install PlatformIO dependencies:

```bash
pio install
```

1. Configure upload port in `platformio.ini` if necessary
2. Build and upload:

```bash
pio run --target upload
```

1. Open serial monitor:

```bash
pio device monitor
```

## Network Access

On boot the PEACH-PIT joins WiFi and is reachable two ways:

- **By name**: `peach-pit.local` (mDNS). Works on most Macs, but not on every machine or wired network.
- **By IP address**: use this for any client that can't resolve `.local` (e.g. the acquisition microscope PC).

The IP address is DHCP-assigned, so it can change between boots. To find it:

- **OLED**: after connecting, the boot screen shows `peach-pit.local` and then the IP address.
- **Serial monitor** (USB or the WiFi bridge on port 6666): type a command and press Enter:
  - `ip` - print the IP address
  - `info` - hostname, IP, gateway, subnet, SSID, signal strength, MAC, service ports
  - `help` - list commands
- The same info is printed at boot and whenever a client connects to the WiFi serial bridge. If the IP changes after a WiFi reconnect, a `WiFi (re)connected, IP address: ...` log line is emitted.

Services (replace `<ip>` with the address above):

| Service | Address |
| --- | --- |
| Serial/log bridge (TCP) | `<ip>:6666` - one client at a time |
| OTA upload | `pio run -t upload --upload-port <ip>` |
| Monitor over WiFi | `pio device monitor --port socket://<ip>:6666` |

If the address keeps changing, ask your network admin for a DHCP reservation using the MAC from `info`.

## Project Structure

```bash
PEACH_PIT/
├── src/
│   ├── main.cpp              # Entry point and FreeRTOS task initialization
│   ├── messaging.h           # System-wide structs and lock-free queue definitions
│   ├── HardwareConfig.h      # Centralized pinout and hardware definitions
│   ├── core/                 # Core system managers
│   │   ├── InputManager.*    # UI and encoder interaction logic
│   │   ├── SequenceEngine.*  # Autonomous step-based sequence runner
│   │   ├── StorageManager.*  # NVS wrapper for saving/loading limits and speeds
│   │   └── SystemState.h     # Global UI state enum and sequence definitions
│   ├── tasks/                # FreeRTOS task implementations (Active Objects)
│   │   ├── ActiveMotionNode.h # Base template for all motion nodes
│   │   ├── ActuatorNode.*    # Linear actuator control loop
│   │   ├── ArmNode.*         # Servo motor arm control loop
│   │   └── MotorNode.*       # Stepper motor Z-axis control loop
│   └── drivers/              # Hardware abstraction layer
│       ├── EncoderDriver.*   # Quadrature encoder interface
│       └── LCDDriver.*       # OLED display driver
├── include/                  # Public headers
├── lib/                      # External libraries
├── test/                     # Unit tests
├── platformio.ini           # PlatformIO project configuration
└── README.md                # This file
```

## Usage

### User Interface

The UI is driven by an encoder with short-press, long-press, and double-press actions:

- **Level 0 (Main Menu)**: Select between Shutdown, Arm, Actuator, Z Motor, and Auto.
- **Level 1 (Sub-Menu)**: Set limits (Top/Mid/Bot or Tip/Buf/Clr), adjust Jog/Go speeds, or clear calibration.

### Sequence Engine

The autonomous sequence engine uses a step-based approach defined in `SystemState.h`:

```cpp
enum SequenceAction {
  SEQ_MOVE_Z,        // Move Z-axis to target limit index (0=Bot, 1=Mid, 2=Top)
  SEQ_MOVE_ARM,      // Move Arm target percent
  SEQ_MOVE_ACTUATOR, // Set actuator to target limit index (0,1,2) with custom speed
  SEQ_WAIT_MS,       // Interruptible delay
  SEQ_WAIT_USER,     // Wait for user button press
  SEQ_MOVE_ARM_AND_Z // Move Arm and Z simultaneously
};
```

Each sequence step can be interrupted by E-STOP events, ensuring safe operation.

## Configuration

Hardware pins and global constants are located in `src/HardwareConfig.h`.
Speeds and limits are dynamically configurable at runtime via the UI and persisted to NVS.

## Task Priorities

FreeRTOS task priorities (higher number = higher priority):

- **EncoderTask** (3): Critical for position feedback
- **Controller** (3): Main UI and input control logic
- **MotorNode** (2): Z-Axis stepper control loop
- **ActuatorNode** (2): Linear actuator positioning
- **ArmNode** (2): Servo angle control
- **LCD** (2): Display refresh

## License

MIT License

Copyright (c) 2026 PEACH PIT Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Project Status

**Active Development** - Current focus on refining the autonomous sequence engine and UI reliability.

### Recent Changes (v3.0)

- **Architecture Refactor**: Replaced shared-memory `controller` blob with `ActiveMotionNode` architecture. Tasks communicate exclusively via lock-free FreeRTOS queues (`messaging.h`).
- **Data-Driven Subsystems**: Extracted `StorageManager`, `InputManager`, and `SequenceEngine`.
- **Dynamic UI Speeds**: Jog and GOTO speeds for all axes are now configurable via the physical Encoder UI and saved to NVS.
- **Dynamic Limits**: `SequenceEngine` now dynamically looks up calibrated physical positions for the Z-Axis and Actuator at runtime, eliminating hardcoded depths.
- **Hardware Abstraction**: Centralized pin definitions to `HardwareConfig.h`.
