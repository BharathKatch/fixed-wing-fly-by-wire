#include <Arduino.h>

#include "Receiver.h"
#include "IMU.h"
#include "Controller.h"
#include "Telemetry.h"

// ---------- Hardware ----------
Receiver receiver;

//Set the Teensy pins that drive each servo
const uint8_t pitchServoPin = 36;
const uint8_t rollServoPin = 22;
const uint8_t yawServoPin = 19;

IMU imu(
    10, // CS
    9,  // INT
    8   // RST
);

Telemetry telemetry(imu);

Controller controller(
    pitchServoPin,
    rollServoPin,
    yawServoPin
);

// ---------- Timing ----------
uint32_t lastLoopMicros = 0;

void setup() {

    Serial.begin(115200);
    Serial.println("Boot");

    receiver.begin();

    if (!imu.begin()) {
        Serial.println("IMU failed!");
    }else{
        Serial.println("IMU Success");
    }

    controller.begin();

    if (!telemetry.begin()) {
        Serial.println("Telemetry unavailable");
    }

    lastLoopMicros = micros();
}

void loop() {

    uint32_t now = micros();

    float dt = (now - lastLoopMicros);
    dt = dt/1000000.f;

    lastLoopMicros = now;

    receiver.update();

    imu.update();

    controller.update(
        receiver,
        imu,
        dt
    );

    telemetry.update();

    // Maintain 200 Hz
    while (micros() - now < 5000) {
    }
}