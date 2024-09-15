#define WEBUI // WiFi and Web UI
// #define SPIFFSEDITOR // include the SPPIFFS editor
// #define INPUT_PS3  // PS3 controller via bluetooth. Dependencies take up quite some program space!
#define INPUT_XBOX // Xbox controller via bluetooth. Dependencies take up quite some program space!
// #define LED_PINS
// #define BATTERY_VOLTAGE

#include <Arduino.h>
#include <Preferences.h> // for storing settings
#include <fastStepper.h>
#include "PID.h"
#ifdef WEBUI
#include "webui.h"

extern plotType plot;
#endif // WEBUI

void parseCommand(char *data, uint8_t length);

extern Preferences preferences;

extern fastStepper motLeft;
extern fastStepper motRight;

// these are all needed so we can plot them in the WebUI
extern float maxStepSpeed;

extern PID pidAngle;
extern PID pidPos;
extern PID pidSpeed;
extern float accAngle;
extern float filterAngle;
extern float gyroFilterConstant;
extern float gyroGain;
extern float pidAngleOutput;
extern float pidPosOutput;
extern float pidSpeedOutput;
extern float speedFilterConstant; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
extern float steerFilterConstant; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
extern char BTaddress[];

/*  Remote control structure
    Every remote should give a speed and steer command from -100 ... 100
    To adjust "driving experience", e.g. a slow beginners mode, or a fast expert mode,
    a gain can be adjusted for the speed and steer inputs.
    Additionaly, a selfRight input can be used. When setting this bit to 1,
    the robot will enable control in an attempt to self right.
    The override input can be used to control the robot when it is lying flat.
    The robot will switch automatically from override to balancing mode, if it happens to right itself.
    The disable control input can be used to
        1) disable the balancing mode
        2) disable the self-right attempt
        3) disable the override mode
    Depending on which state the robot is in.
*/
typedef struct
{
    float speed = 0;
    float steer = 0;
    float speedGain = 0.7;
    float steerGain = 0.6;
    float speedOffset = 0.0;
    bool selfRight = 0;
    bool disableControl = 0;
    bool override = 0;
} remoteControlType;
extern remoteControlType remoteControl;