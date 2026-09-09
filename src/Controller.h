#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <cmath>

#include "Receiver.h"
#include "IMU.h"
#include "MathTypes.h"

enum class ControlMode {
    PASSTHROUGH,
    FBW
};

struct PIDGains {
    float kp;
    float ki;
    float kd;
};

struct PIDState {
    float integral;
    float previousError;
};

class Controller {
public:
    Controller(
        uint8_t pitchServoPin,
        uint8_t rollServoPin,
        uint8_t yawServoPin
    );

    bool begin();

    void update(
        const Receiver& receiver,
        const IMU& imu,
        float dt
    );

    ControlMode getMode() const;

private:
    // Hardware
    Servo pitchServo;
    Servo rollServo;
    Servo yawServo;

    uint8_t pitchServoPin;
    uint8_t rollServoPin;
    uint8_t yawServoPin;

    // Mode state
    ControlMode currentMode;
    ControlMode previousMode;

    // Desired attitude
    Quaternion desiredQuat;

    // PID gains
    PIDGains pitchGains;
    PIDGains rollGains;
    PIDGains yawGains;

    // PID states
    PIDState pitchState;
    PIDState rollState;
    PIDState yawState;

    // Command limits
    float maxRollRateRad;
    float maxPitchRateRad;
    float maxYawRateRad;

    // Safety/output limits
    float maxIntegral;
    float maxServoCorrection;

    //elevon mixer
    void elevonMixer(int* pitch, int* roll);
    
    // Main control flow
    void runPassthrough(const Receiver& receiver);

    void runFBW(
        const Receiver& receiver,
        const IMU& imu,
        float dt
    );

    // Mode/safety logic
    bool imuHealthy(
        const IMU& imu
    ) const;

    ControlMode requestedModeFromReceiver(const Receiver& receiver) const;

    void handleModeTransition(
        const IMU& imu
    );

    void resetForFBW(
        const IMU& imu
    );

    void resetPIDStates();

    // Desired attitude update
    void updateDesiredQuaternion(
        const Receiver& receiver,
        float dt
    );

    // Quaternion error
    Vector3 getAttitudeError(
        const Quaternion& currentQuat
    ) const;

    // PID
    float calculatePID(
        float attitudeError,
        float measuredAngularRate,
        PIDGains gains,
        PIDState& state,
        float dt
    );

    // Servo output
    void writeServoCorrections(
        float pitchCorrection,
        float rollCorrection,
        float yawCorrection
    );

    int correctionToServoMicroseconds(float correction) const;
    int receiverToServoMicroseconds(int16_t receiverValue) const;

    float constrainFloat(
        float value,
        float minValue,
        float maxValue
    ) const;
};