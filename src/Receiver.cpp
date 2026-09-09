#include "Receiver.h"
#include <cmath>

// Change these pins to match actual receiver wiring
const uint8_t Receiver::pins[NUM_CHANNELS] = {
    23,  // Pitch
    37,  // Roll
    18,  // Yaw
    2   // Mode
};

// Set the minimum values that the receiver outputs to each channel for later mapping
const int16_t Receiver::MIN_VALS[NUM_CHANNELS] = {
    1000, //Pitch
    1000, //Roll
    1000, //Yaw
    1000 //Mode
};

// Set maximum values that the receiver will output for later mapping
const int16_t Receiver::MAX_VALS[NUM_CHANNELS] = {
    2000, //Pitch
    2000, //Roll
    2000, //Yaw
    2000 //Mode
};

// Set the values that the receiver sends when sticks are centered (for later mapping)
const int16_t Receiver::CENTER_VALS[NUM_CHANNELS] = {
    1500, //Pitch
    1500, //Roll
    1500, //Yaw
    1500 //Mode
};

volatile uint32_t Receiver::riseTime[NUM_CHANNELS] = {0, 0, 0, 0};

volatile uint16_t Receiver::pulseWidth[NUM_CHANNELS] = {
    1500, 1500, 1500, 1500
};

void Receiver::begin() {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        pinMode(pins[i], INPUT);

        NormalizedVals[i] = 0;
    }

    attachInterrupt(digitalPinToInterrupt(pins[0]), isrPitch, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins[1]), isrRoll, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(pins[2]), isrYaw, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pins[3]), isrMode, CHANGE);
}

// normalizes and applies deadband to receiver values in preparation for reading by Controller
void Receiver::update(){
    for(int i = 0; i < NUM_CHANNELS; i++){
        NormalizedVals[i] = normalizeVals(i);
    }

    NormalizedVals[PITCH_CH] = applyDeadband(PITCH_CH);
    NormalizedVals[ROLL_CH] = applyDeadband(ROLL_CH);
    NormalizedVals[YAW_CH] = applyDeadband(YAW_CH);
}

void Receiver::handleInterrupt(uint8_t channel) {
    if (digitalRead(pins[channel]) == HIGH) {
        riseTime[channel] = micros();
    } else if(digitalRead(pins[channel]) == LOW){
        uint32_t width = micros() - riseTime[channel];

        if (width >= 800 && width <= 2200) {
            pulseWidth[channel] = width;
        }
    }
}

int16_t Receiver::normalizeVals(uint8_t channel)
{
    noInterrupts();
    uint16_t raw = pulseWidth[channel];
    interrupts();

    raw = constrain(raw, MIN_VALS[channel], MAX_VALS[channel]);

    if (raw >= CENTER_VALS[channel]) {
        return map(raw, CENTER_VALS[channel], MAX_VALS[channel], 0, 500);
    } else {
        return map(raw, MIN_VALS[channel], CENTER_VALS[channel], -500, 0);
    }
}

int16_t Receiver::applyDeadband(uint8_t channel){
    if(std::abs(NormalizedVals[channel]) <= 15){
        return 0;
    }

    return NormalizedVals[channel];
}

void Receiver::isrPitch() {
    handleInterrupt(0);
}

void Receiver::isrRoll() {
    handleInterrupt(1);
}

void Receiver::isrYaw() {
    handleInterrupt(2);
}

void Receiver::isrMode() {
    handleInterrupt(3);
}

int16_t Receiver::getPitch() const{
    return NormalizedVals[PITCH_CH];
}

int16_t Receiver::getRoll() const{
    return NormalizedVals[ROLL_CH];
}

int16_t Receiver::getYaw() const{
    return NormalizedVals[YAW_CH];
}

int16_t Receiver::getMode() const{
    return NormalizedVals[MODE_CH];
}