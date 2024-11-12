#include <Arduino.h>
#include "globals.h"

#ifdef LED_LIGHTS

#include "led.h"
#include <FastLED.h>

#define NUM_LEDS 5 // number of LEDs on strip
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 32
// With parallel updates for the LEDs so fast, we get flickering if we call
// FastLED.Show every loop. Maintain a 'dirty' bit so we know when to call Show.
boolean leds_dirty = true;
CRGB leds[NUM_LEDS];

// With parallel updates for the LEDs so fast, we get flickering if we call
// FastLED.Show every loop. Maintain a 'dirty' bit so we know when to call Show.
extern boolean leds_dirty;
extern CRGB leds[NUM_LEDS]; // array of LEDs

#ifdef HEARTBEAT
uint8_t bloodHue = 0x77;     // Blood color [hue from 0-255]
uint8_t bloodSat = 255;      // Blood staturation [0-255]
int flowDirection = -1;      // Use either 1 or -1 to set flow direction
uint16_t cycleLength = 1500; // Lover values = continuous flow, higher values = distinct pulses.
uint16_t pulseLength = 150;  // How long the pulse takes to fade out.  Higher value is longer.
uint16_t pulseOffset = 200;  // Delay before second pulse.  Higher value is more delay.
uint8_t baseBrightness = 10; // Brightness of LEDs when not pulsing. Set to 0 for off.

uint8_t pulseWave8(uint32_t ms, uint16_t cycleLength, uint16_t pulseLength)
{
    uint16_t T = ms % cycleLength;
    if (T > pulseLength)
        return baseBrightness;
    uint16_t halfPulse = pulseLength / 2;
    if (T <= halfPulse)
    {
        return (T * 255) / halfPulse; // first half = going up
    }
    else
    {
        return ((pulseLength - T) * 255) / halfPulse; // second half = going down
    }
}

int sumPulse(int time_shift)
{
    // time_shift = 0;  //Uncomment to heart beat/pulse all LEDs together
    int pulse1 = pulseWave8(millis() + time_shift, cycleLength, pulseLength);
    int pulse2 = pulseWave8(millis() + time_shift + pulseOffset, cycleLength, pulseLength);
    return qadd8(pulse1, pulse2); // Add pulses together without overflow
}

void heartBeat()
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        uint8_t bloodVal = sumPulse((5 / NUM_LEDS / 2) + (NUM_LEDS / 2) * i * flowDirection);
        leds[i] = CHSV(bloodHue, bloodSat, bloodVal);
    }
}

#endif // HEARTBEAT

static float pulseSpeed = 0.75; // Larger value gives faster pulse.

float valueMin = 32.0;  // Pulse minimum value (Should be less then valueMax).
float valueMax = 255.0; // Pulse maximum value (Should be larger then valueMin).

float val = valueMin;                                    // Do Not Edit
static float delta = (valueMax - valueMin) / 2.35040238; // Do Not Edit

void breath()
{
    float dV = ((exp(sin(pulseSpeed * millis() / 2000.0 * PI)) - 0.36787944) * delta);
    val = valueMin + dV;

    EVERY_N_MILLISECONDS(5)
    {
        for (int i = 0; i < LED_BELLY - LED_EYES + 1; i++)
        {
            leds[LED_EYES + i] = CHSV(HUE_PINK, 255, val);
        }
        leds_dirty = true;
    }
}

void LED_setup()
{
    FastLED.addLeds<LED_TYPE, PIN_LED_DATA, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
}

void LED_loop()
{
    breath();
    if (leds_dirty)
    {
        FastLED.show();
        leds_dirty = false;
    }
}

void LED_set(LedLights light, CRGB color)
{
    leds[light] = color;
    leds_dirty = true;
}

#else

void LED_setup() {};
void LED_loop() {};
void LED_set(LedLights light, CRGB color) {};

#endif // LED_LIGHTS
