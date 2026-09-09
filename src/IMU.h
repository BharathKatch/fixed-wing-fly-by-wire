#pragma once

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#include "MathTypes.h"

class IMU {
public:

    IMU(uint8_t csPin, uint8_t intPin, uint8_t rstPin);

    bool begin();
    bool update();

    Quaternion getQuaternion() const;
    Vector3 getGyro() const;

    EulerAngles getEulerRad() const;
    EulerAngles getEulerDeg() const;

    bool attitudeFresh() const;
    bool gyroFresh() const;

private:

    bool initialized;
    
    // Hardware
    uint8_t csPin;
    uint8_t intPin;
    uint8_t rstPin;


    Adafruit_BNO08x bno;
    sh2_SensorValue_t sensorValue;

    // Raw sensor data
    Quaternion qRaw;
    Vector3 gyroRaw;

    // Aircraft-frame data
    Quaternion qAircraft;
    Vector3 gyroAircraft;

    // Mounting correction
    Quaternion qSensorToAircraft;

    // Timestamps
    uint32_t lastQuatMicros;
    uint32_t lastGyroMicros;

    // Internal helpers
    bool enableReports();

    void applyMountingRotation();
};