#define WEBUI // WiFi and Web UI
// #define MDNS // include MDNS support
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

// ESP32 Pin Assignments

// -- Stepper motor pin assignments
#define motEnablePin 19

#define motLeftUStepPin1 18
#define motLeftUStepPin2 05
#define motLeftStepPin 33
#define motLeftDirPin 32

#define motRightUStepPin1 04
#define motRightUStepPin2 27
#define motRightStepPin 26
#define motRightDirPin 25

// -- Others
#define PIN_BATTERY_VOLTAGE 36 // ADC pin connected to voltage divider

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#ifdef LED_PINS
#define PIN_LED 32
#define PIN_LED_LEFT 33
#define PIN_LED_RIGHT 26
#endif // LED_PINS



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