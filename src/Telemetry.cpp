#include "Telemetry.h"

Telemetry::Telemetry(IMU& imu)
    : imu(imu)
{
}

bool Telemetry::begin() {

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD initialization failed");
        return false;
    }

    if (!createLogFile()) {
        Serial.println("Could not create telemetry file");
        return false;
    }

    // CSV header
    const char* header = "time_us,gyro_x,gyro_y,gyro_z\n";
    writeToBuffer(header, strlen(header));

    lastLogTime = micros();

    Serial.println("Telemetry initialized");

    return true;
}


bool Telemetry::createLogFile() {

    char filename[20];

    for (int i = 1; i <= 9999; i++) {

        snprintf(filename, sizeof(filename), "FLIGHT%04d.CSV", i);

        if (!SD.exists(filename)) {

            logFile = SD.open(filename, FILE_WRITE);

            if (logFile) {
                Serial.print("Logging to: ");
                Serial.println(filename);

                return true;
            }

            return false;
        }
    }

    return false;
}


void Telemetry::update() {

    uint32_t now = micros();
    
    uint32_t elapsed = now - lastLogTime;

    if (elapsed < LOG_PERIOD_US) {
        return;
    }

    uint32_t periodsElapsed = elapsed / LOG_PERIOD_US;

    if (periodsElapsed > 1) {
        missedLogSlots += periodsElapsed - 1;
    }

    lastLogTime += periodsElapsed * LOG_PERIOD_US;

    // Don't log stale gyro data
    bool gyroFresh = imu.gyroFresh();

    if (!gyroFresh) {
        staleGyroCount++;
    }

    Vector3 gyro = imu.getGyro();

    char line[128];

    int length = snprintf(
        line,
        sizeof(line),
        "%lu,%.6f,%.6f,%.6f,%d,%lu,%lu,%lu,%lu,%lu\n",
        now,
        gyro.x,
        gyro.y,
        gyro.z,
        gyroFresh ? 1 : 0,
        missedLogSlots,
        staleGyroCount,
        bufferWriteCount,
        filesystemFlushCount,
        sdWriteErrorCount
    );

    if (length > 0) {
        writeToBuffer(line, length);
    }
}


void Telemetry::writeToBuffer(const char* data, size_t length) {

    // If this entry would overflow the buffer, flush first
    if (bufferIndex + length >= BUFFER_SIZE) {
        flushBuffer();
    }

    memcpy(buffer + bufferIndex, data, length);

    bufferIndex += length;
}


void Telemetry::flushBuffer() {

    if (bufferIndex == 0) {
        return;
    }

    if (!logFile) {
        sdWriteErrorCount++;
        return;
    }

    size_t bytesWritten = logFile.write(
        reinterpret_cast<const uint8_t*>(buffer),
        bufferIndex
    );

    if (bytesWritten == bufferIndex) {

        // Successful RAM buffer → SD write
        bufferWriteCount++;
        writesSinceFlush++;

        // Buffer contents are now written
        bufferIndex = 0;

        // Occasionally force filesystem data to SD
        if (writesSinceFlush >= FLUSH_INTERVAL) {
            logFile.flush();

            filesystemFlushCount++;
            writesSinceFlush = 0;
        }

    } else {
        sdWriteErrorCount++;
    }
}


void Telemetry::flush() {

    flushBuffer();

    if (logFile) {
        logFile.flush();
    }
}


void Telemetry::close() {

    flush();

    if (logFile) {
        logFile.close();
    }
}