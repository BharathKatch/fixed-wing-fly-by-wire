#pragma once

#include <Arduino.h>
#include <SD.h>
#include "IMU.h"

class Telemetry {
public:
    Telemetry(IMU& imu);

    bool begin();
    void update();

    void flush();
    void close();

private:
    IMU& imu;

    File logFile;

    static constexpr uint32_t LOG_PERIOD_US = 20000; // 50 Hz
    static constexpr size_t BUFFER_SIZE = 4096;

    char buffer[BUFFER_SIZE];
    size_t bufferIndex = 0;

    uint32_t lastLogTime = 0;

    uint32_t missedLogSlots = 0;
    uint32_t staleGyroCount = 0;
    uint32_t bufferFlushCount = 0;
    uint32_t sdWriteErrorCount = 0;
    uint32_t writesSinceFlush = 0;

    static constexpr uint32_t FLUSH_INTERVAL = 4;
    uint32_t bufferWriteCount = 0;
    uint32_t filesystemFlushCount = 0;

    bool createLogFile();
    void writeToBuffer(const char* data, size_t length);
    void flushBuffer();
};