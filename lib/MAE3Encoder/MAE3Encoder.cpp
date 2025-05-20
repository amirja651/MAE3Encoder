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

MAE3Encoder::MAE3Encoder(uint8_t signalPin, uint8_t encoderId)
    : signalPin(signalPin), encoderId(encoderId), state{}, lastPulseTime(0), newPulseAvailable(false), pulseStartTime(0)
{
}

bool MAE3Encoder::begin()
{
    if (encoderId >= MAX_ENCODERS)
    {
        return false;
    }

    // Configure pins
    pinMode(signalPin, INPUT_PULLUP);

    // Store instance for interrupt handling
    encoderInstances[encoderId] = this;

    // Configure interrupt based on encoder ID
    switch (encoderId)
    {
        case 0:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler0, CHANGE);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler1, CHANGE);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler2, CHANGE);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(signalPin), interruptHandler3, CHANGE);
            break;
    }

    // Initialize state
    state.current_Pulse = 0;
    state.laps          = 0;
    state.direction     = Direction::UNKNOWN;
    state.last_pulse    = 0;

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
            if (lastRisingEdgeTime - lastFallingEdgeTime > (max_t - 1))
            {
                lastRisingEdgeTime = 0;
                return;
            }
            state.t_off = lastRisingEdgeTime - lastFallingEdgeTime;
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
            if (lastFallingEdgeTime - lastRisingEdgeTime > (max_t - 1))
            {
                lastFallingEdgeTime = 0;
                return;
            }
            state.t_on = lastFallingEdgeTime - lastRisingEdgeTime;
        }

        unsigned long total_t = state.t_on + state.t_off;
        state.total_t         = total_t;

        if (total_t == 0)
        {
            return;
        }

        unsigned long x_measured = (state.t_on * max_t) / total_t - 1;

        if (x_measured <= (max_t - 3))
        {
        }
        else if (x_measured == (max_t - 1))
        {
            x_measured = (max_t - 3);
        }
        else
        {
            return;
        }

        /*if (std::abs(static_cast<int32_t>(x_measured - state.current_Pulse)) <= 2)
        {
            return;
        }*/

        state.current_Pulse = x_measured;
        newPulseAvailable   = true;
        lastPulseTime       = currentTime;
    }
}

void MAE3Encoder::update()
{
    if (!newPulseAvailable)
    {
        return;
    }

    Direction newDirection = Direction::UNKNOWN;

    /*if (std::abs(static_cast<int32_t>(state.current_Pulse - state.last_pulse)) <= 1)
    {
        return;
    }*/

    // Handle wrap-around cases
    uint32_t diff = state.current_Pulse - state.last_pulse;
    if (state.current_Pulse > state.last_pulse)
    {
        if (diff > ((max_t - 1) / 2))
        {
            newDirection = Direction::COUNTER_CLOCKWISE;
        }
        else
        {
            newDirection = Direction::CLOCKWISE;
        }
    }
    else
    {
        if (-diff > ((max_t - 1) / 2))
        {
            newDirection = Direction::CLOCKWISE;
        }
        else
        {
            newDirection = Direction::COUNTER_CLOCKWISE;
        }
    }

    // Handle lap counting
    if (newDirection != Direction::UNKNOWN)
    {
        // Check for clockwise lap completion
        if (newDirection == Direction::CLOCKWISE)
        {
            if (state.last_pulse > ((max_t - 1) * 3 / 4) && state.current_Pulse < ((max_t - 1) / 4))
            {
                state.laps++;
            }
        }
        // Check for counter-clockwise lap completion
        else if (newDirection == Direction::COUNTER_CLOCKWISE)
        {
            if (state.last_pulse < ((max_t - 1) / 4) && state.current_Pulse > ((max_t - 1) * 3 / 4))
            {
                state.laps--;
            }
        }
    }

    // Update state
    state.direction   = newDirection;
    state.last_pulse  = state.current_Pulse;
    newPulseAvailable = false;
}

Direction MAE3Encoder::detectDirection()
{
    uint32_t noiseThreshold = (max_t - 1) * 6 / 1000;
    if (std::abs(static_cast<int32_t>(state.current_Pulse - state.last_pulse)) < noiseThreshold)
    {
        return state.direction;
    }

    // Handle wrap-around cases
    uint32_t diff;
    if (state.current_Pulse > state.last_pulse)
    {
        diff = state.current_Pulse - state.last_pulse;
        if (diff > ((max_t - 1) / 2))
        {
            return Direction::COUNTER_CLOCKWISE;
        }
        return Direction::CLOCKWISE;
    }
    else
    {
        diff = state.last_pulse - state.current_Pulse;
        if (diff > ((max_t - 1) / 2))
        {
            return Direction::CLOCKWISE;
        }
        return Direction::COUNTER_CLOCKWISE;
    }
}

void MAE3Encoder::reset()
{
    state.current_Pulse = 0;
    state.laps          = 0;
    state.direction     = Direction::UNKNOWN;
    state.last_pulse    = 0;
}
