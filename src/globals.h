#ifndef GLOBALS_H
#define GLOBALS_H

//#define TMC2209 // TMC2209 stepper controller library (currently broken with V2 hardware)

// only include the web and winsock servers for debug buids as it only needed during development
#ifdef DEBUG
#define WEBSERVER
#define SERIALINPUT
#define BATTERY_VOLTAGE
#endif // DEBUG

// #define MDNS // include MDNS support
// #define SPIFFSEDITOR // include the SPPIFFS editor
// #define INPUT_PS3  // PS3 controller via bluetooth. Dependencies take up quite some program space!
#define INPUT_XBOX // Xbox controller via bluetooth. Dependencies take up quite some program space!
// #define LED_PINS

#include <Arduino.h>
#include <Preferences.h>
#include <fastStepper.h>
#include "PID.h"
#ifdef WEBSERVER
#include "webserver.h"
#endif // WEBSERVER

// ESP32 Pin Assignments

// Stepper motor pin assignments
#define motEnablePin 27

#define motLeftUStepPin1 19
#define motLeftUStepPin2 18
#define motLeftStepPin 26
#define motLeftDirPin 25

#define motRightUStepPin1 05
#define motRightUStepPin2 04
#define motRightStepPin 33
#define motRightDirPin 32

// TMC2209 Stepper driver
#ifdef TMC2209
#define SERIAL2_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define SERIAL2_RX_PIN 16    // Specify Serial2 RX pin as the default has changed
#define SERIAL2_TX_PIN 17    // Specify Serial2 TX pin as the default has changed
#endif                       // TMC2209

// -- Others
#define PIN_BATTERY_VOLTAGE 36 // ADC pin connected to voltage divider

#define PIN_I2C_SDA 21       // MPU SDA pin
#define PIN_I2C_SCL 22       // MPU SCL pin
#define PIN_MPU_INTERRUPT 23 // MPU interrupt pin

extern Preferences preferences;

extern fastStepper motLeft;
extern fastStepper motRight;

#ifdef SERIALINPUT
void parseCommand(char *data, uint8_t length);
#endif // SERIALINPUT

extern float filterAngle;
extern float gyroFilterConstant;
extern float gyroGain;

// these are all needed so we can plot them in the WebUI
#ifdef BATTERY_VOLTAGE
extern uint32_t tNowMs;
#endif // BATTERY_VOLTAGE
#ifdef WEBSERVER
extern plotType plot;
extern float maxStepSpeed;

extern PID pidAngle;
extern PID pidPos;
extern PID pidSpeed;
extern float accAngle;
extern float pidAngleOutput;
extern float pidPosOutput;
extern float pidSpeedOutput;
extern float speedAlphaConstant; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
extern float steerAlphaConstant; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
#endif                           // WEBSERVER

/*  Remote control structure
    Every remote should give a speed and steer command from -100 ... 100
    To adjust "driving experience", e.g. a slow beginners mode, or a fast expert mode, a gain can be adjusted for the speed and steer inputs.
    Additionaly, a selfRight input can be used. When setting this bit to true, the robot will enable control in an attempt to self right.
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
    bool selfRight = false;
    bool disableControl = false;
    bool override = false;
} remoteControlType;
extern remoteControlType remoteControl;
#endif // GLOBALS_H
