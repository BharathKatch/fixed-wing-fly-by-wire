#pragma once
#include <Arduino.h>
#include <cstdint>

//Define the names of receiver channels
enum ReceiverChannel {
    PITCH_CH = 0,
    ROLL_CH = 1,
    YAW_CH = 2,
    MODE_CH = 3,
};

class Receiver{
public:
    void begin();

    void update();

    int16_t normalizeVals(uint8_t channel);

    int16_t applyDeadband(uint8_t channel);

    int16_t getPitch() const;
    int16_t getRoll() const;
    int16_t getYaw() const;
    int16_t getMode() const;

private:
    static const uint8_t NUM_CHANNELS = 4;

    static const uint8_t pins[NUM_CHANNELS];

    static const int16_t MIN_VALS[NUM_CHANNELS];
    static const int16_t MAX_VALS[NUM_CHANNELS];
    static const int16_t CENTER_VALS[NUM_CHANNELS];

    int16_t NormalizedVals[NUM_CHANNELS];

    static volatile uint32_t riseTime[NUM_CHANNELS];
    static volatile uint16_t pulseWidth[NUM_CHANNELS];

    static void handleInterrupt(uint8_t channel);

    static void isrPitch();
    static void isrRoll();
    static void isrYaw();
    static void isrMode();
};