#include <MAE3Encoder.h>
#include <inttypes.h>

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

MAE3Encoder encoder = MAE3Encoder(ENC_A, 0);

void setup()
{
    Serial.begin(115200);
    delay(1000);
    while (!Serial)
    {
        delay(10);
    }

    encoder.begin();
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
        // gotoRowCol(1, 1);
        //  table header
        Serial.printf("Laps\tPulse_Width\tCurrent_Pulse\tDegrees\t\tDirection\tPulse_High\tPulse_Low\tTotal_Pulse\n");

        // table data
        Serial.printf("%d\t", state.laps);
        Serial.printf("%ld\t\t", state.pulse_width);
        Serial.printf("%ld\t\t", state.current_Pulse);
        Serial.printf("%.2f\t\t", degrees);
        Serial.printf("%s\t\t", direction.c_str());
        Serial.printf("%ld\t\t", state.t_on);
        Serial.printf("%ld\t\t", state.t_off);
        Serial.printf("%ld\n", state.total_t);
        lastPulseWidthUs = state.current_Pulse;
    }

    delay(100);
}