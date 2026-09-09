# Fixed-Wing Fly-By-Wire Flight Controller

Custom flight controller developed for an experimental fixed-wing RC aircraft using a Teensy 4.1.

## Features

- Quaternion-based attitude control
- Pilot rate-command input
- PID attitude stabilization
- PWM receiver input
- Elevon mixing
- BNO085 IMU integration
- 200 Hz flight-control loop
- 50 Hz servo output
- Buffered SD telemetry logging
- Flight modes including passthrough and fly-by-wire

## Hardware

- Teensy 4.1
- Adafruit BNO085 IMU
- Spektrum RC receiver
- Fixed-wing elevon-controlled aircraft

## Software

Built using PlatformIO and the Arduino framework.

## Architecture

Pilot Input → Desired Angular Rate → Desired Attitude → Quaternion Error → PID Controller → Elevon Mixer → Servos

## Other Notes

Certain lines within Controller.cpp have been commented out to support a flying wing configuration.