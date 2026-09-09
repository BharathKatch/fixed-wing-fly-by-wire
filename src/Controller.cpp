#include <Arduino.h>
#include "Controller.h"
#include "MathUtils.h"

Controller::Controller(
    uint8_t pitchServoPin,
    uint8_t rollServoPin,
    uint8_t yawServoPin
) {
    this->pitchServoPin = pitchServoPin;
    this->rollServoPin = rollServoPin;
    this->yawServoPin = yawServoPin;

    currentMode = ControlMode::PASSTHROUGH;
    previousMode = ControlMode::PASSTHROUGH;

    desiredQuat = {1.0f, 0.0f, 0.0f, 0.0f};

    pitchGains = {1000.0f, 50.0f, 0.1f};
    rollGains  = {1000.0f, 50.0f, 0.1f};
    yawGains   = {0.0f, 0.0f, 0.0f};

    pitchState = {0.0f, 0.0f};
    rollState  = {0.0f, 0.0f};
    yawState   = {0.0f, 0.0f};

    maxPitchRateRad = 2.6f;
    maxRollRateRad  = 5.2f;
    maxYawRateRad   = 1.0f;

    maxIntegral = 100.0f;
    maxServoCorrection = 500.0f;
}

bool Controller::begin() {
    pitchServo.attach(pitchServoPin);
    rollServo.attach(rollServoPin);
    yawServo.attach(yawServoPin);

    pitchServo.writeMicroseconds(1500);
    rollServo.writeMicroseconds(1500);
    yawServo.writeMicroseconds(1500);

    return true;
}

void Controller::update(
    const Receiver& receiver,
    const IMU& imu,
    float dt
) {
    previousMode = currentMode;

    ControlMode requestedMode = requestedModeFromReceiver(receiver);

    if (
        requestedMode == ControlMode::FBW &&
        imuHealthy(imu)
    ) {
        currentMode = ControlMode::FBW;
    } else {
        currentMode = ControlMode::PASSTHROUGH;
    }

    handleModeTransition(imu);

    if (currentMode == ControlMode::FBW) {
        runFBW(receiver, imu, dt);
    } else {
        runPassthrough(receiver);
    }
}

ControlMode Controller::getMode() const {
    return currentMode;
}

//Mode, safety, reset stuff --------------------------------
bool Controller::imuHealthy(const IMU& imu) const {
    return imu.attitudeFresh() && imu.gyroFresh();
}

ControlMode Controller::requestedModeFromReceiver(
    const Receiver& receiver
) const {
    int16_t modeValue = receiver.getMode();

    if (modeValue >= -500 && modeValue <= -200) {
        return ControlMode::FBW;
    }

    return ControlMode::PASSTHROUGH;
}

void Controller::handleModeTransition(const IMU& imu) {
    if (previousMode == ControlMode::PASSTHROUGH &&
        currentMode == ControlMode::FBW) {
        resetForFBW(imu);
    }

    if (
        previousMode == ControlMode::FBW &&
        currentMode == ControlMode::PASSTHROUGH
    ) {
        resetPIDStates();
    }
}

void Controller::resetForFBW(const IMU& imu) {
    desiredQuat = imu.getQuaternion();
    resetPIDStates();
}

void Controller::resetPIDStates() {
    pitchState.integral = 0.0f;
    pitchState.previousError = 0.0f;

    rollState.integral = 0.0f;
    rollState.previousError = 0.0f;

    yawState.integral = 0.0f;
    yawState.previousError = 0.0f;
}

void Controller::elevonMixer(int* pitch, int* roll)
{
    int pitchCmd = *pitch - 1500;
    int rollCmd  = *roll - 1500;

    *pitch = 1500 + pitchCmd - rollCmd;  // left elevon
    *roll  = 1500 + pitchCmd + rollCmd;  // right elevon
}

//Passthrough ---------------------------------
void Controller::runPassthrough(const Receiver& receiver) {
    int pitchUs = receiverToServoMicroseconds(receiver.getPitch());
    int rollUs  = receiverToServoMicroseconds(receiver.getRoll());
    //int yawUs   = receiverToServoMicroseconds(receiver.getYaw());

    //block for elevon mixer for flying wing, comment out for conventional layout
    elevonMixer(&pitchUs, &rollUs);
    pitchUs = constrain(pitchUs, 1000, 2000);
    rollUs = constrain(rollUs, 1000, 2000);

    pitchServo.writeMicroseconds(pitchUs);
    rollServo.writeMicroseconds(rollUs);
    //yawServo.writeMicroseconds(yawUs);
}

int Controller::receiverToServoMicroseconds(int16_t value) const {
    value = constrain(value, -500, 500);
    return 1500 + value;
}

//PID --------------------------------
float Controller::calculatePID(
    float error,
    float angularRate,
    PIDGains gains,
    PIDState& state,
    float dt
) {

    if (dt <= 0.0f) {
        return 0.0f;
    }

    state.integral += error * dt;
    state.integral = constrainFloat(
        state.integral,
        -maxIntegral,
        maxIntegral
    );

    float pTerm = gains.kp * error;
    float iTerm = gains.ki * state.integral;

    // Rate damping from gyro.
    float dTerm = -gains.kd * angularRate;

    state.previousError = error;

    return (pTerm + iTerm + dTerm);
}

//Quaternion Error --------------------------------
Vector3 Controller::getAttitudeError(
    const Quaternion& currentQuat
) const {
    Quaternion qError = MathUtils::multiplyQuat(
        MathUtils::conjugateQuat(currentQuat),
        desiredQuat
    );
    qError = MathUtils::normalizeQuat(qError);

    if (qError.w < 0.0f) {
        qError.w = -qError.w;
        qError.x = -qError.x;
        qError.y = -qError.y;
        qError.z = -qError.z;
    }

    qError.w = constrainFloat(qError.w, -1.0f, 1.0f);

    float angle = 2.0f * acosf(qError.w);

    float sinHalfAngle = sqrtf(
        1.0f - qError.w * qError.w
    );

    if (sinHalfAngle < 0.0001f) {
        return {0.0f, 0.0f, 0.0f};
    }

    Vector3 axis = {
        qError.x / sinHalfAngle,
        qError.y / sinHalfAngle,
        qError.z / sinHalfAngle
    };

    return {
        axis.x * angle,
        axis.y * angle,
        axis.z * angle
    };
}

//Desired Quaternion Integral --------------------------------
void Controller::updateDesiredQuaternion(
    const Receiver& receiver,
    float dt
) {
    if (dt <= 0.0f) {
        return;
    }

    float rollInput  = receiver.getRoll()  / 500.0f;
    float pitchInput = receiver.getPitch() / 500.0f;
    float yawInput   = receiver.getYaw()   / 500.0f;

    float rollRateCmd  = rollInput  * maxRollRateRad;
    float pitchRateCmd = pitchInput * maxPitchRateRad;
    float yawRateCmd   = yawInput   * maxYawRateRad;

    Vector3 rateCmd = {
        rollRateCmd,
        pitchRateCmd,
        yawRateCmd
    };

    float angle = sqrtf(
        rateCmd.x * rateCmd.x +
        rateCmd.y * rateCmd.y +
        rateCmd.z * rateCmd.z
    ) * dt;

    if (angle < 0.000001f) {
        return;
    }

    Vector3 axis = {
        rateCmd.x / (angle / dt),
        rateCmd.y / (angle / dt),
        rateCmd.z / (angle / dt)
    };

    float halfAngle = 0.5f * angle;

    Quaternion deltaQuat = {
        cosf(halfAngle),
        axis.x * sinf(halfAngle),
        axis.y * sinf(halfAngle),
        axis.z * sinf(halfAngle)
    };

    desiredQuat = MathUtils::normalizeQuat(
        MathUtils::multiplyQuat(
            desiredQuat,
            MathUtils::conjugateQuat(deltaQuat)
        )
    );
}

//FBW Output --------------------------------
void Controller::runFBW(
    const Receiver& receiver,
    const IMU& imu,
    float dt
) {
    updateDesiredQuaternion(receiver, dt);

    Quaternion currentQuat = imu.getQuaternion();
    Vector3 gyro = imu.getGyro();

    Vector3 attitudeError = getAttitudeError(currentQuat);

    float rollCorrection = -calculatePID(
        attitudeError.x,
        gyro.x,
        rollGains,
        rollState,
        dt
    );

    float pitchCorrection = -calculatePID(
        attitudeError.y,
        gyro.y,
        pitchGains,
        pitchState,
        dt
    );

    float yawCorrection = calculatePID(
        attitudeError.z,
        gyro.z,
        yawGains,
        yawState,
        dt
    );

    writeServoCorrections(
        pitchCorrection,
        rollCorrection,
        yawCorrection
    );  
}

//Servo Correction ---------------------------------
void Controller::writeServoCorrections(
    float pitchCorrection,
    float rollCorrection,
    float yawCorrection
) {
    int pitchUs = correctionToServoMicroseconds(pitchCorrection);
    int rollUs  = correctionToServoMicroseconds(rollCorrection);
    //int yawUs   = correctionToServoMicroseconds(yawCorrection);

    //block for elevon mixer for flying wing, comment out for conventional layout
    elevonMixer(&pitchUs, &rollUs);
    pitchUs = constrain(pitchUs, 1000, 2000);
    rollUs = constrain(rollUs, 1000, 2000);

    pitchServo.writeMicroseconds(pitchUs);
    rollServo.writeMicroseconds(rollUs);
    //yawServo.writeMicroseconds(yawUs);
}

int Controller::correctionToServoMicroseconds(float correction) const {
    correction = constrainFloat(
        correction,
        -maxServoCorrection,
        maxServoCorrection
    );

    return 1500 + static_cast<int>(correction);
}

float Controller::constrainFloat(
    float value,
    float minimum,
    float maximum
) const {
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}