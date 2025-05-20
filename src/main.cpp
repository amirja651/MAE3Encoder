#include <MAE3Encoder.h>
#include <inttypes.h>

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

MAE3Encoder encoder = MAE3Encoder(ENC_A, 0);

// Buffer for storing output
char outputBuffer[200];

void setup()
{
    Serial.begin(115200);
    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

    encoder.begin();
    printf("\e[1;1H\e[2J");  // clear screen
}

uint32_t lastPulseWidthUs = 0;
float    lastPosition     = 0;

void loop()
{
    encoder.update();
    const auto& state     = encoder.getState();
    String      direction = state.direction == Direction::CLOCKWISE ? "CW" : "CCW";
    float       degrees   = encoder.getPositionDegrees();

    if (fabs(state.current_Pulse - lastPulseWidthUs) > 1)
    {
        printf("\e[2J\e[1;1H");  // clear screen
        // table header
        printf("Laps\tPulse_Width\tCurrent_Pulse\tDegrees\t\tDirection\tPulse_High\tPulse_Low\tTotal_Pulse\n");

        // Format all values into the buffer
        snprintf(outputBuffer, sizeof(outputBuffer), "%d\t%ld\t\t%ld\t\t%.2f\t\t%s\t\t%ld\t\t%ld\t\t%ld\n", state.laps,
                 state.pulse_width, state.current_Pulse, degrees, direction.c_str(), state.t_on, state.t_off, state.total_t);

        // Print the entire buffer at once
        printf(outputBuffer);

        lastPulseWidthUs = state.current_Pulse;
    }

    delay(100);
}