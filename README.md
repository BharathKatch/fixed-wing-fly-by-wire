# Fixed-Wing Fly-By-Wire Flight Controller



Custom fly-by-wire flight controller designed, implemented, and flight-tested on an experimental fixed-wing RC aircraft using a Teensy 4.1 and BNO085 IMU. The system uses quaternion-based attitude control to translate pilot commands into stabilized elevon outputs at 200 Hz.

## Features

- Quaternion-based attitude control
- Pilot rate-command input
- PID attitude stabilization
- PWM receiver input
- Elevon mixing
- BNO085 IMU integration
- 200 Hz flight-control loop
- Buffered SD telemetry logging
- Flight modes including passthrough and fly-by-wire

## Hardware

- Teensy 4.1
- Adafruit BNO085 IMU
- Spektrum RC receiver
- Fixed-wing elevon-controlled aircraft

## Software

Built using PlatformIO and the Arduino framework.

- 200 Hz deterministic flight-control loop
- Buffered SD telemetry logging
- Hardware watchdog configured with a 0.5 s timeout
- Following a watchdog reset, the controller enters a recovery mode that bypasses non-essential
  initialization and defaults to passthrough control, minimizing the time
  required to restore direct pilot authority.

## Architecture

Pilot Input → Desired Angular Rate → Integrate Desired Attitude → Quaternion Attitude Error → PID Controller → Elevon Mixer → Servos

## Flight Testing

The aircraft has been successfully flown in passthrough mode. PID tuning of the Fly-By-Wire system is still in progress, however momentary control of the aircraft within FBW mode has been achieved.

## Other Notes

A detailed engineering portfolio covering the system architecture, control implementation, hardware design, reliability features, and flight testing is available here:
