#include "IMU.h"
#include "MathUtils.h"


IMU::IMU(uint8_t csPin, uint8_t intPin, uint8_t rstPin)
    : initialized(false),
      csPin(csPin),
      intPin(intPin),
      rstPin(rstPin),
      bno(rstPin),
      qRaw{1.0f, 0.0f, 0.0f, 0.0f},
      gyroRaw{0.0f, 0.0f, 0.0f},
      qAircraft{1.0f, 0.0f, 0.0f, 0.0f},
      gyroAircraft{0.0f, 0.0f, 0.0f},
      qSensorToAircraft{1.0f, 0.0f, 0.0f, 0.0f},
      lastQuatMicros(0),
      lastGyroMicros(0)
{
}

bool IMU::begin() {
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    pinMode(intPin, INPUT_PULLUP);

    SPI.begin();

    if (!bno.begin_SPI(csPin, intPin)) {
        initialized = false;
        return false;
    }

    bno.enableReport(SH2_ROTATION_VECTOR, 5000);     // 200 Hz
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);     // 200 Hz

    initialized = true;
    return true;
}

bool IMU::enableReports() {
    bool success = true;

    // 5000 us = 200 Hz
    success &= bno.enableReport(SH2_ROTATION_VECTOR, 5000);
    success &= bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);

    return success;
}

bool IMU::update() {
    if (!initialized) {
        return false;
    }

    // BNO08x INT is usually active-low.
    // If INT is high, no data is ready.
    if (digitalRead(intPin) == HIGH) {
        return false;
    }

    sh2_SensorValue_t sensorValue;

    if (!bno.getSensorEvent(&sensorValue)) {
        return false;
    }

    switch (sensorValue.sensorId) {
        case SH2_ROTATION_VECTOR:
            qRaw = {
                sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k
            };

            qAircraft = MathUtils::multiplyQuat(
                MathUtils::multiplyQuat(qSensorToAircraft, qRaw),
                MathUtils::conjugateQuat(qSensorToAircraft)
            );

            lastQuatMicros = micros();
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            gyroRaw = {
                sensorValue.un.gyroscope.x,
                sensorValue.un.gyroscope.y,
                sensorValue.un.gyroscope.z
            };

            gyroAircraft = MathUtils::rotateVector(qSensorToAircraft, gyroRaw);

            lastGyroMicros = micros();
            break;
    }

    return true;
}

void IMU::applyMountingRotation() {
    // Sensor:   X right, Y forward, Z up
    // Aircraft: X forward, Y right, Z down
    gyroAircraft = {
        gyroRaw.y,
        gyroRaw.x,
        -gyroRaw.z
    };

    qAircraft = MathUtils::normalizeQuat(
        MathUtils::multiplyQuat(
            qRaw,
            qSensorToAircraft
        )
    );
}

Quaternion IMU::getQuaternion() const {
    return qAircraft;
}

Vector3 IMU::getGyro() const {
    return gyroAircraft;
}

bool IMU::attitudeFresh() const {
    return (micros() - lastQuatMicros) < 20000;
}

bool IMU::gyroFresh() const {
    return (micros() - lastGyroMicros) < 20000;
}