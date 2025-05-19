#include "MAE3Encoder.h"
#include <algorithm>

// Initialize static member
MAE3Encoder* MAE3Encoder::encoderInstances[MAX_ENCODERS] = {nullptr};

// Individual interrupt handlers for each encoder
void IRAM_ATTR MAE3Encoder::interruptHandler0()
{
    if (encoderInstances[0])
    {
        encoderInstances[0]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler1()
{
    if (encoderInstances[1])
    {
        encoderInstances[1]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler2()
{
    if (encoderInstances[2])
    {
        encoderInstances[2]->processInterrupt();
    }
}

void IRAM_ATTR MAE3Encoder::interruptHandler3()
{
    if (encoderInstances[3])
    {
        encoderInstances[3]->processInterrupt();
    }
}

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t interruptPin, uint8_t encoderId)
    : signalPin(signalPin),
      interruptPin(interruptPin),
      encoderId(encoderId),
      state{},
      lastPulseTime(0),
      newPulseAvailable(false),
      pulseStartTime(0),
      constants(ENCODER_12BIT)
{
}

bool MAE3Encoder::begin()
{
    if (encoderId >= MAX_ENCODERS)
    {
        return false;
    }

    // Configure pins
    pinMode(signalPin, INPUT);
    pinMode(interruptPin, INPUT);

    // Store instance for interrupt handling
    encoderInstances[encoderId] = this;

    // Configure interrupt based on encoder ID
    switch (encoderId)
    {
        case 0:
            attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(interruptPin), interruptHandler3, CHANGE);
            break;
    }

    // Initialize state
    state.currentPulse = 0;
    state.laps         = 0;
    state.direction    = Direction::UNKNOWN;
    state.lastPulse    = 0;

    return true;
}

void IRAM_ATTR MAE3Encoder::processInterrupt()
{
    static unsigned long lastFallingEdgeTime = 0;
    static unsigned long lastRisingEdgeTime  = 0;
    unsigned long        currentTime         = micros();

    if (digitalRead(signalPin) == HIGH)
    {
        // Rising edge
        lastRisingEdgeTime = currentTime;
        if (lastFallingEdgeTime != 0)
        {
            // Measure low pulse width (falling to rising)
            state.pulseWidthLowUs = lastRisingEdgeTime - lastFallingEdgeTime;
        }
        pulseStartTime = currentTime;
    }
    else
    {
        // Falling edge
        lastFallingEdgeTime = currentTime;
        if (lastRisingEdgeTime != 0)
        {
            // Measure high pulse width (rising to falling)
            state.pulseWidthHighUs = lastFallingEdgeTime - lastRisingEdgeTime;
        }

        uint32_t x        = (state.pulseWidthHighUs * 4098) / (state.pulseWidthHighUs + state.pulseWidthLowUs) - 1;
        uint32_t Position = 0;

        if (x <= 4094)
        {
            Position = x;
        }
        else if (x == 4096)
        {
            Position = 4095;
        }
        else
        {
            return;
        }
        
        state.currentPulse = Position;
        newPulseAvailable  = true;
        lastPulseTime      = currentTime;
    }
}

bool MAE3Encoder::update()
{
    if (!newPulseAvailable)
    {
        return false;
    }

    // Adjust noise threshold based on resolution (0.6% of full scale)
    uint32_t noiseThreshold = constants.PULSE_PER_REV * 6 / 1000;
    if (std::abs(static_cast<int32_t>(state.currentPulse - state.lastPulse)) < noiseThreshold)
    {
        return false;
    }

    // Calculate direction before updating state
    Direction newDirection = detectDirection();

    // Handle lap counting
    if (newDirection != Direction::UNKNOWN)
    {
        // Check for clockwise lap completion
        if (newDirection == Direction::CLOCKWISE)
        {
            if (state.lastPulse > (constants.PULSE_PER_REV * 3 / 4) && state.currentPulse < (constants.PULSE_PER_REV / 4))
            {
                state.laps++;
            }
        }
        // Check for counter-clockwise lap completion
        else if (newDirection == Direction::COUNTER_CLOCKWISE)
        {
            if (state.lastPulse < (constants.PULSE_PER_REV / 4) && state.currentPulse > (constants.PULSE_PER_REV * 3 / 4))
            {
                state.laps--;
            }
        }
    }

    // Update state
    state.direction   = newDirection;
    state.lastPulse   = state.currentPulse;
    newPulseAvailable = false;
    return true;
}

Direction MAE3Encoder::detectDirection()
{
    uint32_t noiseThreshold = constants.PULSE_PER_REV * 6 / 1000;
    if (std::abs(static_cast<int32_t>(state.currentPulse - state.lastPulse)) < noiseThreshold)
    {
        return state.direction;
    }

    // Handle wrap-around cases
    uint32_t diff;
    if (state.currentPulse > state.lastPulse)
    {
        diff = state.currentPulse - state.lastPulse;
        if (diff > (constants.PULSE_PER_REV / 2))
        {
            return Direction::COUNTER_CLOCKWISE;
        }
        return Direction::CLOCKWISE;
    }
    else
    {
        diff = state.lastPulse - state.currentPulse;
        if (diff > (constants.PULSE_PER_REV / 2))
        {
            return Direction::CLOCKWISE;
        }
        return Direction::COUNTER_CLOCKWISE;
    }
}

void MAE3Encoder::reset()
{
    state.currentPulse = 0;
    state.laps         = 0;
    state.direction    = Direction::UNKNOWN;
    state.lastPulse    = 0;
}
