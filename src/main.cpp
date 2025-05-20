#include <MAE3Encoder.h>
#include <inttypes.h>

static const uint16_t ENC_A = 36;
static const uint16_t ENC_B = 39;
static const uint16_t ENC_C = 34;
static const uint16_t ENC_D = 35;

MAE3Encoder encoder = MAE3Encoder(ENC_A, 0);

// helper for table print
#define CLEAR_SCREEN printf("\e[1;1H\e[2J")

void gotoRowCol(const int row, const int col)
{
    // Position the cursor at the desired position (row, col)
    Serial.print("\033[");  // Begin of escape sequence
    Serial.print(row);      // row number (begins with 1)
    Serial.print(";");
    Serial.print(col);  // column (begins with 1)
    Serial.print("H");
}

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
        printf("Laps\tPulse_Width\tCurrent_Pulse\tDegrees\t\tDirection\tPulse_High\tPulse_Low\n");

        // table data
        Serial.printf("%d\t", state.laps);
        Serial.printf("%ld\t\t", state.pulse_width);
        Serial.printf("%ld\t\t", state.current_Pulse);
        Serial.printf("%.2f\t\t", degrees);
        Serial.printf("%s\t\t", direction.c_str());
        Serial.printf("%ld\t\t", state.t_on);
        Serial.printf("%ld\n", state.t_off);
        lastPulseWidthUs = state.current_Pulse;
    }

    delay(100);
}