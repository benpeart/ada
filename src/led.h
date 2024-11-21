#ifndef LED_H
#define LED_H

#include <FastLED.h>

typedef enum
{
    LED_BATTERY = 0,
    LED_ENABLED,
    LED_CONTROLLER_CONNECTED,
    LED_EYES,
    LED_EARS,
    LED_BELLY,
    NUM_LEDS        // number of LEDs
} LedLights;

void LED_setup();
void LED_loop();
void LED_set(LedLights light, CRGB color);

#endif // LED_H
