# MAE3Encoder Library

A library for interfacing with MAE3 encoders on ESP32 platform.

## Features

- Support for up to 4 encoders simultaneously
- Interrupt-based pulse detection
- Direction detection (Clockwise/Counter-clockwise)
- Position tracking in multiple units (degrees, mm, μm)
- Lap counting
- Noise filtering
- Pulse width measurement

## Installation

1. Download the library
2. Place it in your Arduino libraries folder or PlatformIO lib folder
3. Include it in your project

## Usage

```cpp
#include <MAE3Encoder.h>

// Define encoder pins
static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

// Create encoder instance
MAE3Encoder encoder = MAE3Encoder(ENC_A, 0);

void setup() {
    Serial.begin(115200);
    encoder.begin();
}

void loop() {
    encoder.update();
    const auto& state = encoder.getState();
    
    // Get position in degrees
    float degrees = encoder.getPositionDegrees();
    
    // Get position in millimeters
    float mm = encoder.getPositionMM();
    
    // Get position in micrometers
    float um = encoder.getPositionUM();
    
    // Get total travel in mm
    float totalTravel = encoder.getTotalTravelMM();
}
```

## API Reference

### Constructor
```cpp
MAE3Encoder(uint8_t signalPin, uint8_t encoderId)
```

### Methods
- `bool begin()`: Initialize the encoder
- `void update()`: Update encoder state
- `const EncoderState& getState()`: Get current encoder state
- `float getPositionDegrees()`: Get position in degrees
- `float getPositionMM()`: Get position in millimeters
- `float getPositionUM()`: Get position in micrometers
- `float getTotalTravelMM()`: Get total travel in millimeters
- `void reset()`: Reset encoder state

### Constants
- `LEAD_SCREW_PITCH_MM`: Lead screw pitch in mm (0.5mm)
- `TOTAL_TRAVEL_MM`: Total travel distance in mm (30mm)
- `max_t`: Maximum pulse count (4120)

## License

This library is licensed under the MIT License. 