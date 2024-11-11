#ifndef LED_H
#define LED_H

#include <FastLED.h>

typedef enum
{
    LED_ENABLED = 0,
    LED_CONTROLLER_CONNECTED,
    LED_EYES,
    LED_EARS,
    LED_BELLY
} LedLights;

void LED_setup();
void LED_loop();
void LED_set(LedLights light, CRGB color);

#endif // LED_H
