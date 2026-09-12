#include <Arduino.h>
#include <Watchdog_t4.h>

WDT_T4<WDT1> wdt;

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

//Watchdog reset conditions
bool recoveryMode = false;

// ---------- Timing ----------
constexpr uint32_t LOOP_PERIOD_US = 5000;
uint32_t lastLoopMicros = 0;

void setup() {
    //Determine reset cause/boot mode first
    uint32_t resetCause = SRC_SRSR;

    if (resetCause & SRC_SRSR_WDOG_RST_B) {
        recoveryMode = true;
    }

    if (recoveryMode) {
        //Only starts what's necessary, and defaults to passthrough since IMU never starts
        receiver.begin();

        controller.begin();
    }else{
        Serial.begin(115200);

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

    }

    // Start watchdog
    WDT_timings_t config;

    config.timeout = 1;   // 1 second for initial testing

    wdt.begin(config);

    //Serial will be off in recovery mode
    if(!recoveryMode){
        Serial.println("Watchdog started");
    }

    lastLoopMicros = micros();
}

void loop() {

    uint32_t now = micros();

    float dt = (now - lastLoopMicros) / 1000000.f;
    lastLoopMicros = now;


    receiver.update();


    // IMU will be off in recovery mode
    if (!recoveryMode) {
        imu.update();
    }


    controller.update(
        receiver,
        imu,
        dt
    );


    // Only run telemetry if system is healthy
    if (!recoveryMode) {
        telemetry.update();
    }


    // How long did the entire frame take?
    uint32_t computeTime = micros() - now;


    // Only feed watchdog if entire frame finished on time
    if (computeTime < LOOP_PERIOD_US) {
        wdt.feed();
    }


    // Maintain 200 Hz
    while (micros() - now < LOOP_PERIOD_US) {
    }
}