#ifndef MAE3_ENCODER2_H
#define MAE3_ENCODER2_H

#include <Arduino.h>
#include <array>
#include <cstdlib>  // For abs with uint32_t

// Maximum number of encoders supported
constexpr uint8_t MAX_ENCODERS = 4;

// Constants for both 10-bit and 12-bit versions
struct EncoderConstants
{
    // Pulse width constraints (converted to nanoseconds to avoid floating point in ISR)
    uint32_t MIN_PULSE_WIDTH_MIN_NS;  // Minimum valid pulse width in nanoseconds
    uint32_t MIN_PULSE_WIDTH_TYP_NS;  // Typical pulse width in nanoseconds
    uint32_t MIN_PULSE_WIDTH_MAX_NS;  // Maximum minimum pulse width in nanoseconds

    uint32_t MAX_PULSE_WIDTH_MIN_NS;  // Minimum maximum pulse width in nanoseconds
    uint32_t MAX_PULSE_WIDTH_TYP_NS;  // Typical maximum pulse width in nanoseconds
    uint32_t MAX_PULSE_WIDTH_MAX_NS;  // Maximum valid pulse width in nanoseconds

    // Original floating point values for non-ISR calculations
    float MIN_PULSE_WIDTH_MIN;  // Minimum valid pulse width (minimum value μs)
    float MIN_PULSE_WIDTH_TYP;  // Minimum valid pulse width (typical value μs)
    float MIN_PULSE_WIDTH_MAX;  // Minimum valid pulse width (maximum value μs)

    float MAX_PULSE_WIDTH_MIN;  // Maximum valid pulse width (minimum value μs)
    float MAX_PULSE_WIDTH_TYP;  // Maximum valid pulse width (typical value μs)
    float MAX_PULSE_WIDTH_MAX;  // Maximum valid pulse width (maximum value μs)

    // Period and frequency constraints
    uint32_t PULSE_PERIOD_MIN;  // Total period (minimum value)
    uint32_t PULSE_PERIOD_TYP;  // Total period (typical value)
    uint32_t PULSE_PERIOD_MAX;  // Total period (maximum value)

    float PWM_FREQUENCY_MIN;  // PWM frequency (minimum value)
    float PWM_FREQUENCY_TYP;  // PWM frequency (typical value)
    float PWM_FREQUENCY_MAX;  // PWM frequency (maximum value)

    // Resolution and delay values
    uint32_t PULSE_PER_REV;      // Resolution (1024 for 10-bit, 4096 for 12-bit)
    uint32_t PROPAGATION_DELAY;  // Propagation delay in microseconds
};

// Constants for 12-bit encoder (MAE3-P12)
constexpr EncoderConstants ENCODER_12BIT = {
    // Nanosecond values for ISR (converted from μs to ns)
    .MIN_PULSE_WIDTH_MIN_NS = 950,   // 0.95 μs from datasheet MIN (950 ns)
    .MIN_PULSE_WIDTH_TYP_NS = 1000,  // 1.00 μs from datasheet TYP (1000 ns)
    .MIN_PULSE_WIDTH_MAX_NS = 1050,  // 1.05 μs from datasheet MAX (1050 ns)

    .MAX_PULSE_WIDTH_MIN_NS = 3892000,  // 3892 μs from datasheet MIN (3892000 ns)
    .MAX_PULSE_WIDTH_TYP_NS = 4097000,  // 4097 μs from datasheet TYP (4097000 ns)
    .MAX_PULSE_WIDTH_MAX_NS = 4302000,  // 4302 μs from datasheet MAX (4302000 ns)

    // Original float values in microseconds
    .MIN_PULSE_WIDTH_MIN = 0.95f,  // 0.95 μs from datasheet MIN
    .MIN_PULSE_WIDTH_TYP = 1.00f,  // 1.00 μs from datasheet TYP
    .MIN_PULSE_WIDTH_MAX = 1.05f,  // 1.05 μs from datasheet MAX

    .MAX_PULSE_WIDTH_MIN = 3892.0f,  // 3892 μs from datasheet MIN
    .MAX_PULSE_WIDTH_TYP = 4097.0f,  // 4097 μs from datasheet TYP
    .MAX_PULSE_WIDTH_MAX = 4302.0f,  // 4302 μs from datasheet MAX

    // Total period values (calculated from frequency)
    .PULSE_PERIOD_MIN = 3731,  // Calculated from 268 Hz frequency (1000000/268)
    .PULSE_PERIOD_TYP = 4098,  // Calculated from 244 Hz frequency (1000000/244)
    .PULSE_PERIOD_MAX = 4545,  // Calculated from 220 Hz frequency (1000000/220)

    // PWM frequency values (in Hz)
    .PWM_FREQUENCY_MIN = 220.0f,  // 220 Hz from datasheet MIN
    .PWM_FREQUENCY_TYP = 244.0f,  // 244 Hz from datasheet TYP
    .PWM_FREQUENCY_MAX = 268.0f,  // 268 Hz from datasheet MAX

    // Resolution and propagation delay
    .PULSE_PER_REV     = 4096,  // 12-bit resolution (2^12)
    .PROPAGATION_DELAY = 384    // 384 μs from datasheet
};

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
    uint32_t  currentPulse;      // Current pulse value
    int32_t   laps;              // Number of complete rotations
    Direction direction;         // Current direction of rotation
    uint32_t  lastPulse;         // Last valid pulse value
    uint32_t  pulseWidthUs;      // Pulse width in microseconds
    uint32_t  pulseWidthHighUs;  // High pulse width (rising to falling)
    uint32_t  pulseWidthLowUs;   // Low pulse width (falling to rising)
};

class MAE3Encoder
{
public:
    MAE3Encoder(uint8_t signalPin, uint8_t interruptPin, uint8_t encoderId);

    bool begin();

    bool update();

    const EncoderState& getState() const
    {
        return state;
    }

    // Degrees per pulse
    float getDegreesPerPulse() const
    {
        return 360.0f / constants.PULSE_PER_REV;
    }

    // Millimeters per pulse
    float getMMPerPulse() const
    {
        return LEAD_SCREW_PITCH_MM / constants.PULSE_PER_REV;
    }

    // Micrometers per pulse
    float getUMPerPulse() const
    {
        return getMMPerPulse() * 1000.0f;
    }

    // Position in degrees
    float getPositionDegrees() const
    {
        return state.currentPulse * getDegreesPerPulse();
    }

    // Position in mm
    float getPositionMM() const
    {
        return state.currentPulse * getMMPerPulse();
    }

    // Position in μm
    float getPositionUM() const
    {
        return state.currentPulse * getUMPerPulse();
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
    const uint8_t interruptPin;
    const uint8_t encoderId;

    // State management
    EncoderState state;

    // Filtering and timing
    unsigned long lastPulseTime;

    // Interrupt handling
    volatile bool          newPulseAvailable;
    volatile unsigned long pulseStartTime;

    // Encoder configuration
    const EncoderConstants& constants;

    // Individual interrupt handler for this encoder
    static void IRAM_ATTR interruptHandler0();
    static void IRAM_ATTR interruptHandler1();
    static void IRAM_ATTR interruptHandler2();
    static void IRAM_ATTR interruptHandler3();

    // Static array to store encoder instances for interrupt handling
    static MAE3Encoder* encoderInstances[4];
};

#endif  // MAE3_ENCODER2_H