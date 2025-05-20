#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <cstdlib>  // For abs with uint32_t

// Maximum number of encoders supported
constexpr uint8_t MAX_ENCODERS = 4;

// Linear motion constants
constexpr float LEAD_SCREW_PITCH_MM = 0.5f;      // Lead screw pitch in mm
constexpr float TOTAL_TRAVEL_MM     = 30.0f;     // Total travel distance in mm
constexpr float LEAD_SCREW_PITCH_UM = 500.0f;    // 0.5mm = 500μm
constexpr float TOTAL_TRAVEL_UM     = 30000.0f;  // 30mm = 30000μm

// Direction enum
enum class Direction
{
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    UNKNOWN
};

struct EncoderState
{
    volatile unsigned long current_Pulse;  // Current pulse value
    int32_t                laps;           // Number of complete rotations
    Direction              direction;      // Current direction of rotation
    volatile unsigned long last_pulse;     // Last valid pulse value
    volatile unsigned long pulse_width;    // Pulse width in microseconds
    volatile unsigned long t_on;           // High pulse width (rising to falling)
    volatile unsigned long t_off;          // Low pulse width (falling to rising)
};

class MAE3Encoder
{
public:
    MAE3Encoder(uint8_t signalPin, uint8_t encoderId);

    bool begin();

    void update();

    const EncoderState& getState() const
    {
        return state;
    }

    // Degrees per pulse
    float getDegreesPerPulse() const
    {
        return 360.0f / 4097;
    }

    // Millimeters per pulse
    float getMMPerPulse() const
    {
        return LEAD_SCREW_PITCH_MM / 4097;
    }

    // Micrometers per pulse
    float getUMPerPulse() const
    {
        return getMMPerPulse() * 1000.0f;
    }

    // Position in degrees
    float getPositionDegrees() const
    {
        return state.current_Pulse * getDegreesPerPulse();
    }

    // Position in mm
    float getPositionMM() const
    {
        return state.current_Pulse * getMMPerPulse();
    }

    // Position in μm
    float getPositionUM() const
    {
        return state.current_Pulse * getUMPerPulse();
    }

    // Total travel in mm
    float getTotalTravelMM() const
    {
        float totalDistance = (state.laps * LEAD_SCREW_PITCH_MM) + getPositionMM();
        return std::min(totalDistance, TOTAL_TRAVEL_MM);
    }

    // Total travel in μm
    float getTotalTravelUM() const
    {
        float totalDistanceUM = (state.laps * LEAD_SCREW_PITCH_UM) + getPositionUM();
        return std::min(totalDistanceUM, TOTAL_TRAVEL_UM);
    }

    void reset();

    void processInterrupt();

private:
    // Median filter implementation for noise reduction
    uint32_t medianFilter();

    // Direction detection
    Direction detectDirection();

    // Performance optimization
    void optimizeInterrupt();

    // Pin assignments
    const uint8_t signalPin;
    const uint8_t encoderId;

    // State management
    EncoderState state;

    // Filtering and timing
    unsigned long lastPulseTime;

    // Interrupt handling
    volatile bool          newPulseAvailable;
    volatile unsigned long pulseStartTime;

    // Individual interrupt handler for this encoder
    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder* encoderInstances[4];
};

#endif  // MAE3_ENCODER2_H