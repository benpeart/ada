/*
A high speed balancing robot, running on an ESP32.

Forked and highly modified by Ben Peart. See http://github.com/benpeart/ada

Wouter Klop
wouter@elexperiment.nl
For updates, see elexperiment.nl

Use at your own risk. This code is far from stable.

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
This basically means: if you use my code, acknowledge it.
Also, you have to publish all modifications.

*/

#include <Arduino.h>
#include "globals.h"
#include "debug.h"
#ifdef WEBUI
#include "webui.h"
#endif // WEBUI
#include <gyro.h>
#ifdef TMC2209
#include <TMCStepper.h>
#endif // TMC2209
#ifdef INPUT_PS3
#include "ps3.h"
#endif // INPUT_PS3
#ifdef INPUT_XBOX
#include "xbox.h"
#endif // INPUT_XBOX

// Use 'Expodential Smoothing' to improve driving behaviour by preventing abrupt changes in speed or direction
float speedAlphaConstant = 0.1; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
float steerAlphaConstant = 0.1; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)

// TMC2209 Stepper driver
#ifdef TMC2209
#define DRIVER_ADDRESS_LEFT 0b00  // TMC2209 Driver address according to MS1=LOW and MS2=LOW
#define DRIVER_ADDRESS_RIGHT 0b01 // TMC2209 Driver address according to MS1=HIGH and MS2=LOW

#define SERIAL2_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define SERIAL_BAUD_RATE 500000
#define SERIAL2_RX_PIN 16 // Specify Serial2 RX pin as the default has changed
#define SERIAL2_TX_PIN 17 // Specify Serial2 TX pin as the default has changed

#define MAX_CURRENT 2000 // in mA; should match capabilities of stepper motor
#define R_SENSE 0.11f    // Match to your driver
                         // SilentStepStick series use 0.11
                         // UltiMachine Einsy and Archim2 boards use 0.2
                         // Panucatt BSD2660 uses 0.1
                         // Watterott TMC5160 uses 0.075

TMC2209Stepper tmcDriverLeft(&SERIAL2_PORT, R_SENSE, DRIVER_ADDRESS_LEFT);   // Hardware Serial
TMC2209Stepper tmcDriverRight(&SERIAL2_PORT, R_SENSE, DRIVER_ADDRESS_RIGHT); // Hardware Serial
#else
// #define STEPPER_DRIVER_A4988 // Use A4988 stepper driver, which uses different microstepping settings
#define STEPPER_DRIVER_TMC2209 // Use TMC2209 stepper driver, which uses different microstepping settings

#endif // TMC2209

// Remote control structure
remoteControlType remoteControl;

// ----- Function prototypes
void parseSerial();
void setMicroStep(uint8_t uStep);
void sendConfigurationData(uint8_t num);

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();

// -- EEPROM
Preferences preferences;
#define PREF_VERSION 1 // if setting structure has been changed, count this number up to delete all settings
#define PREF_NAMESPACE "pref"
#define PREF_KEY_VERSION "ver"

// -- Stepper motor
fastStepper motLeft(motLeftStepPin, motLeftDirPin, 0, motLeftTimerFunction);
fastStepper motRight(motRightStepPin, motRightDirPin, 1, motRightTimerFunction);

uint8_t microStep = 0;
float maxStepSpeed = 1500;

// -- PID control
#define dT_MICROSECONDS 5000
float dT = dT_MICROSECONDS / 1000000.0;

#define PID_ANGLE 0
#define PID_POS 1
#define PID_SPEED 2

#define PID_ANGLE_MAX 12
PID pidAngle(cPID, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
#define PID_POS_MAX 35
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
PID pidSpeed(cP, dT, PID_POS_MAX, -PID_POS_MAX);

// make these global so we can use them in webui.cpp
float pidAngleOutput;
float pidPosOutput;
float pidSpeedOutput;

enum controlType
{
    ANGLE_ONLY = 0,
    ANGLE_PLUS_POSITION,
    ANGLE_PLUS_SPEED
};
controlType controlMode = ANGLE_PLUS_POSITION;

// Threshold for fall detection. If integral of error of angle controller is larger than this value, controller is disabled
#define angleErrorIntegralThreshold 30.0
#define angleErrorIntegralThresholdDuringSelfright angleErrorIntegralThreshold * 3
#define angleEnableThreshold 5.0   // If (absolute) robot angle is below this threshold, enable control
#define angleDisableThreshold 70.0 // If (absolute) robot angle is above this threshold, disable control (robot has fallen down)

// ----- Interrupt functions -----
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR motLeftTimerFunction()
{
    portENTER_CRITICAL_ISR(&timerMux);
    motLeft.timerFunction();
    portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR motRightTimerFunction()
{
    portENTER_CRITICAL_ISR(&timerMux);
    motRight.timerFunction();
    portEXIT_CRITICAL_ISR(&timerMux);
}

// optimize unnecessary calls to digitalWrite as it is relatively slow
void motEnable(boolean enable)
{
    static boolean enabled = -1; // ensure the first time we call motEnable, it will actually set the pin correctly

    if (enabled != enable)
    {
        digitalWrite(motEnablePin, !enable); // Inverted action on enable pin
        enabled = enable;
    }
}

// ----- Main code
void setup()
{
    Serial.begin(115200);

    // Init EEPROM, if not done before
    preferences.begin(PREF_NAMESPACE, false); // false = RW-mode
    if (preferences.getUInt(PREF_KEY_VERSION, 0) != PREF_VERSION)
    {
        preferences.clear(); // Remove all preferences under the opened namespace
        preferences.putUInt(PREF_KEY_VERSION, PREF_VERSION);
        DB_PRINTF("EEPROM init complete, all preferences deleted, new pref_version: %d\n", PREF_VERSION);
    }

#ifdef LED_PINS
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED_LEFT, OUTPUT);
    pinMode(PIN_LED_RIGHT, OUTPUT);
    digitalWrite(PIN_LED, 0);
    digitalWrite(PIN_LED_LEFT, 1); // Turn on one LED to indicate we are live
    digitalWrite(PIN_LED_RIGHT, 0);
#endif // LED_PINS

    // Disable steppers during startup
    pinMode(motEnablePin, OUTPUT);
    motEnable(false);

#ifdef TMC2209
    // use TMC2209 pins MS1 and MS2 to set the correct address for Serial control
    pinMode(motLeftUStepPin1, OUTPUT);
    pinMode(motLeftUStepPin2, OUTPUT);
    digitalWrite(motLeftUStepPin1, LOW);
    digitalWrite(motLeftUStepPin2, LOW);
    pinMode(motRightUStepPin1, OUTPUT);
    pinMode(motRightUStepPin2, OUTPUT);
    digitalWrite(motRightUStepPin1, HIGH);
    digitalWrite(motRightUStepPin2, LOW);

    // setup a TMC2209 stepper driver
    SERIAL2_PORT.begin(SERIAL_BAUD_RATE, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN); // Initialize HW UART drivers with the correct RX/TX pins
    tmcDriverLeft.begin();                                                            // Initiate pins and registeries
    tmcDriverRight.begin();                                                           // Initiate pins and registeries
    tmcDriverLeft.toff(5);                                                            // Enables driver in software (0 = driver off, 1-15 = driver enabled)
    tmcDriverRight.toff(5);                                                           // Enables driver in software (0 = driver off, 1-15 = driver enabled)

    // test the serial connections to the TMC2209
    uint8_t resultLeft = tmcDriverLeft.test_connection();
    if (resultLeft != 0)
    {
        DB_PRINTF("tmcDriverLeft.test_connection failed with error code: %d\n", resultLeft);
    }
    uint8_t resultRight = tmcDriverRight.test_connection();
    if (resultRight != 0)
    {
        DB_PRINTF("tmcDriverRight.test_connection failed with error code: %d\n", resultRight);
    }
    if (resultLeft == 0 && resultRight == 0)
    {
        tmcDriverLeft.rms_current(MAX_CURRENT);  // Set stepper current to MAX_CURRENT in mA
        tmcDriverRight.rms_current(MAX_CURRENT); // Set stepper current to MAX_CURRENT in mA

        // configure microstepping
        tmcDriverLeft.mstep_reg_select(true);  // Microstep resolution selected by MRES register
        tmcDriverRight.mstep_reg_select(true); // Microstep resolution selected by MRES register

        // enable CoolStep to save up to 75% of energy
        tmcDriverLeft.TCOOLTHRS(0xFFFF);  // Set threshold for CoolStep
        tmcDriverRight.TCOOLTHRS(0xFFFF); // Set threshold for CoolStep
                                          //    tmcDriverLeft.THIGH(0);                  // Set high threshold (not avaialble with TMC2209)
                                          //    tmcDriverRight.THIGH(0);                 // Set high threshold (not avaialble with TMC2209)
        tmcDriverLeft.SGTHRS(10);         // Set StallGuard threshold
        tmcDriverRight.SGTHRS(10);        // Set StallGuard threshold
        tmcDriverLeft.semin(5);           // Minimum current adjustment min = 1, max = 15
        tmcDriverRight.semin(5);          // Minimum current adjustment min = 1, max = 15
        tmcDriverLeft.semax(2);           // Maximum current adjustment min = 0, max = 15, 0-2 recommended
        tmcDriverRight.semax(2);          // Maximum current adjustment min = 0, max = 15, 0-2 recommended
        tmcDriverLeft.sedn(0b01);         // Current down step speed
        tmcDriverRight.sedn(0b01);        // Current down step speed

        // StealthChop2 guarantees that the motor is absolutely quiet in standstill and in slow motion
        // SpreadCycle provides high dynamics (reacting at once to a change of motor velocity) and highest peak velocity at low vibration
        tmcDriverLeft.en_spreadCycle(false);  // Toggle spreadCycle on TMC2208/2209/2224. false = StealthChop (low speed) / true = SpreadCycle (faster speeds)
        tmcDriverRight.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224. false = StealthChop (low speed) / true = SpreadCycle (faster speeds)
        tmcDriverLeft.pwm_autoscale(true);    // Needed for stealthChop
        tmcDriverRight.pwm_autoscale(true);   // Needed for stealthChop
    }

#else
    // setup micro steping pins
    pinMode(motLeftUStepPin1, OUTPUT);
    pinMode(motLeftUStepPin2, OUTPUT);
    pinMode(motRightUStepPin1, OUTPUT);
    pinMode(motRightUStepPin2, OUTPUT);

#endif // TMC2209

    // setup stepper motors
    setMicroStep(16);
    motLeft.init();
    motRight.init();

    // Gyro setup
    Gyro_setup();

#ifdef WEBUI
    WebUI_setup();
#endif // WEBUI

    pidAngle.setParameters(0.65, 1.0, 0.075, 15);
    pidPos.setParameters(1, 0, 1.2, 50);
    pidSpeed.setParameters(6, 5, 0, 20);

// Setup PS3 controller
#ifdef INPUT_PS3
    ps3_setup();
#endif

// Setup Xbox controller
#ifdef INPUT_XBOX
    Xbox_setup();
#endif

    DB_PRINTLN("Booted, ready for driving!");
#ifdef LED_PINS
    digitalWrite(PIN_LED_RIGHT, 1);
#endif // LED_PINS
}

void loop()
{
    static unsigned long tLast = 0;
    float avgMotSpeed;
    static float smoothedSteer = 0;
    static float smoothedSpeed = 0;
    static boolean enableControl = false;
    static float avgMotSpeedSum = 0;
    int32_t avgMotStep;
    static float avgBatteryVoltage = 0;
    static uint32_t lastInputTime = 0;
    uint32_t tNowMs;
    float absSpeed = 0;
    static boolean overrideMode = false, lastOverrideMode = false;
    static boolean selfRight = false;
    static boolean disableControl = false;
    static float angleErrorIntegral = 0;

    unsigned long tNow = micros();
    tNowMs = millis();

    if (tNow - tLast > dT_MICROSECONDS)
    {
        tLast = tNow;
        Gyro_ReadSensor();

        if (remoteControl.selfRight && !enableControl)
        { // Start self-right action (stops when robot is upright)
            selfRight = true;
            disableControl = false;
            remoteControl.selfRight = false; // Reset single action bool
        }
        else if (remoteControl.disableControl && enableControl)
        { // Sort of kill-switch
            disableControl = true;
            selfRight = false;
            remoteControl.disableControl = false;
        }

        // Use 'Expodential Smoothing' to improve driving behaviour by preventing abrupt changes in speed or direction
        // Scale speed down to 20% of the remoteControl.speed value as it is way to fast to control otherwise
        // TODO: may want to dampen steering input as smoothedSpeed increases
        smoothedSpeed = speedAlphaConstant * remoteControl.speed / 5.0 + (1 - speedAlphaConstant) * smoothedSpeed;
        smoothedSteer = steerAlphaConstant * remoteControl.steer + (1 - steerAlphaConstant) * smoothedSteer;

        if (enableControl)
        {
            if (abs(smoothedSpeed) < 0.2)
            {
                // remoteControl.speed = 0;
            }
            else
            {
                lastInputTime = tNowMs;
                if (controlMode == ANGLE_PLUS_POSITION)
                {
                    DB_PRINTLN("control mode: ANGLE_PLUS_SPEED");
                    controlMode = ANGLE_PLUS_SPEED;
                    motLeft.setStep(0);
                    motRight.setStep(0);
                    pidSpeed.reset();
                }
            }

            // Switch to position control if no input is received for a certain amount of time
            if (tNowMs - lastInputTime > 2000 && controlMode == ANGLE_PLUS_SPEED)
            {
                DB_PRINTLN("control mode: ANGLE_PLUS_POSITION");
                controlMode = ANGLE_PLUS_POSITION;
                motLeft.setStep(0);
                motRight.setStep(0);
                pidPos.reset();
            }

            // Actual controller computations
            if (controlMode == ANGLE_ONLY)
            {
                pidAngle.setpoint = smoothedSpeed * 2;
            }
            else if (controlMode == ANGLE_PLUS_POSITION)
            {
                avgMotStep = (motLeft.getStep() + motRight.getStep()) / 2;
                pidPos.setpoint = smoothedSpeed;
                pidPos.input = -((float)avgMotStep) / 1000.0;
                pidPosOutput = pidPos.calculate();
                pidAngle.setpoint = pidPosOutput;
            }
            else if (controlMode == ANGLE_PLUS_SPEED)
            {
                pidSpeed.setpoint = smoothedSpeed;
                pidSpeed.input = -avgMotSpeedSum / 100.0;
                pidSpeedOutput = pidSpeed.calculate();
                pidAngle.setpoint = pidSpeedOutput;
            }

            // calculate the correction needed to hit our target setpoint
            pidAngle.input = filterAngle;
            pidAngleOutput = pidAngle.calculate();

            avgMotSpeedSum += pidAngleOutput / 2;
            avgMotSpeedSum = constrain(avgMotSpeedSum, -maxStepSpeed, maxStepSpeed);
            avgMotSpeed = avgMotSpeedSum;

            // add in steering input
            motLeft.speed = avgMotSpeed + smoothedSteer;
            motRight.speed = avgMotSpeed - smoothedSteer;

            // Detect if robot has fallen. Concept: integrate angle controller error over time.
            // If absolute integrated error surpasses threshold, disable controller
            angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
            if (selfRight)
            {
                if (abs(angleErrorIntegral) > angleErrorIntegralThresholdDuringSelfright)
                {
                    selfRight = false;
                    disableControl = true;
                }
            }
            else
            {
                if (abs(angleErrorIntegral) > angleErrorIntegralThreshold)
                {
                    disableControl = true;
                }
            }

            // Dynamically switch microstepping to achieve even more insane speeds
#ifdef DYNAMIC_MICROSTEPPING
            absSpeed = abs(avgMotSpeed);

            if (absSpeed > (150 * 32 / microStep) && microStep > 1)
                microStep /= 2;
            if (absSpeed < (130 * 32 / microStep) && microStep < 32)
                microStep *= 2;

            setMicroStep(microStep);
#endif // DYNAMIC_MICROSTEPPING

            // Disable control if robot is almost horizontal. Re-enable if upright.
            if ((abs(filterAngle) > angleDisableThreshold && !selfRight) || disableControl)
            {
                DB_PRINTLN("control disabled");
                enableControl = false;
                // disableControl = 0; // Reset disableControl flag
                motLeft.speed = 0;
                motRight.speed = 0;
                motEnable(true);
#ifdef LED_PINS
                digitalWrite(PIN_LED_LEFT, 0);
                digitalWrite(PIN_LED_RIGHT, 0);
#endif // LED_PINS
            }
            if (abs(filterAngle) < angleEnableThreshold && selfRight)
            {
                selfRight = false;
                angleErrorIntegral = 0; // Reset, otherwise the fall detection will be triggered immediately
            }
        }
        else
        { // Control not active

            // Override control
            if (overrideMode && !lastOverrideMode)
            { // Transition from disable to enable
                // Enable override mode
                motLeft.speed = 0;
                motRight.speed = 0;
                motEnable(true); // Enable motors
            }
            else if (!overrideMode && lastOverrideMode)
            {
                motEnable(false); // disable motors
                overrideMode = false;
            }
            lastOverrideMode = overrideMode;

            if (abs(filterAngle) > angleEnableThreshold + 5)
            { // Only reset disableControl flag if angle is out of "enable" zone, otherwise robot will keep cycling between enable and disable states
                disableControl = false;
            }

            if ((abs(filterAngle) < angleEnableThreshold || selfRight) && !disableControl)
            { // (re-)enable and reset stuff
                DB_PRINTLN("control enabled");
                enableControl = true;
#ifdef LED_PINS
                digitalWrite(PIN_LED_LEFT, 1);
                digitalWrite(PIN_LED_RIGHT, 1);
#endif // LED_PINS
                DB_PRINTLN("control mode: ANGLE_PLUS_POSITION");
                controlMode = ANGLE_PLUS_POSITION;

                if (!overrideMode)
                {
                    avgMotSpeedSum = 0;
                    motEnable(true); // Enable motors
                    pidAngle.reset();
                }
                else
                {
                    avgMotSpeedSum = (motLeft.speed + motRight.speed) / 2;
                    overrideMode = false;
                }

                motLeft.setStep(0);
                motRight.setStep(0);
                pidPos.reset();
                pidSpeed.reset();

                angleErrorIntegral = 0;
            }

            if (overrideMode)
            {
                motLeft.speed = -30 * smoothedSpeed + 2 * smoothedSteer;
                motRight.speed = -30 * smoothedSpeed - 2 * smoothedSteer;

                // Run angle PID controller in background, such that it matches when controller takes over, if needed
                pidAngle.input = filterAngle;
                pidAngleOutput = pidAngle.calculate();
            }
            // Serial << motLeft.speed << "\t" << motRight.speed << "\t" << overrideMode << endl;
        }

        motLeft.update();
        motRight.update();

#ifdef WEBUI
        WebUI_loop();
#endif // WEBUI
    }

    parseSerial();

    // Handle PS3 controller
#ifdef INPUT_PS3
    Ps3_loop();
#endif

    // Handle Xbox controller
#ifdef INPUT_XBOX
    Xbox_loop();
#endif
}

void parseSerial()
{
    static char serialBuf[63];
    static uint8_t pos = 0;
    char currentChar;

    while (Serial.available())
    {
        currentChar = Serial.read();
        serialBuf[pos++] = currentChar;
        if (currentChar == 'x')
        {
            parseCommand(serialBuf, pos);
            pos = 0;
            while (Serial.available())
                Serial.read();
            memset(serialBuf, 0, sizeof(serialBuf));
        }
    }
}

void parseCommand(char *data, uint8_t length)
{
    float val2;
    if ((data[length - 1] == 'x') && length >= 3)
    {
        switch (data[0])
        {
        case 'c':
        { // Change controller parameter
            uint8_t controllerNumber = data[1] - '0';
            char cmd2 = data[2];
            float val = atof(data + 3);

            // Make pointer to PID controller
            PID *pidTemp;
            switch (controllerNumber)
            {
            case 1:
                pidTemp = &pidAngle;
                break;
            case 2:
                pidTemp = &pidPos;
                break;
            case 3:
                pidTemp = &pidSpeed;
                break;
            }

            switch (cmd2)
            {
            case 'p':
                pidTemp->K = val;
                break;
            case 'i':
                pidTemp->Ti = val;
                break;
            case 'd':
                pidTemp->Td = val;
                break;
            case 'n':
                pidTemp->N = val;
                break;
            case 't':
                pidTemp->controllerType = (uint8_t)val;
                break;
            case 'm':
                pidTemp->maxOutput = val;
                break;
            case 'o':
                pidTemp->minOutput = -val;
                break;
            }
            pidTemp->updateParameters();

            // Serial << controllerNumber << "\t" << pidTemp->K << "\t" << pidTemp->Ti << "\t" << pidTemp->Td << "\t" << pidTemp->N << "\t" << pidTemp->controllerType << endl;
            break;
        }
        case 'a': // Change angle offset
            angleOffset = atof(data + 1);
            DB_PRINTLN(angleOffset);
            break;
        case 'f':
            gyroFilterConstant = atof(data + 1);
            DB_PRINTLN(gyroFilterConstant);
            break;
        case 'm':
            val2 = atof(data + 1);
            DB_PRINTLN(val2);
            controlMode = (controlType)val2;
            break;
        case 'u':
            setMicroStep(atoi(data + 1));
            break;
        case 'g':
            gyroGain = atof(data + 1);
            break;
#ifdef WEBUI
        case 'p':
        {
            switch (data[1])
            {
            case 'e':
                plot.enable = atoi(data + 2);
                break;
            case 'p':
                plot.prescaler = atoi(data + 2);
                break;
            }
            break;
        }
#endif // WEBUI
        case 'j':
            gyroGain = atof(data + 1);
            break;
        case 'k':
        {
            uint8_t cmd2 = atoi(data + 1);
            if (cmd2 == 1)
            { // calibrate gyro
                Gyro_CalculateOffset(100);
            }
            else if (cmd2 == 2)
            { // calibrate acc
                DB_PRINTF("Updating angle offset from %.2f to %.2f", angleOffset, filterAngle);
                angleOffset = filterAngle;
                preferences.putFloat("angle_offset", angleOffset);
            }
            break;
        }
        case 'l':
            maxStepSpeed = atof(&data[1]);
            break;
        case 'n':
            gyroFilterConstant = atof(&data[1]);
            break;
        case 'w':
        {
            char cmd2 = data[1];
            char buf[63];
            uint8_t len;

            switch (cmd2)
            {
            case 'r':
                DB_PRINTLN("Rebooting...");
                ESP.restart();
                // pidParList.sendList(&wsServer);
                break;
#ifdef WEBUI
            case 'l': // Send wifi networks to WS client
                sendWifiList();
                break;
            case 's': // Update WiFi SSID
                len = length - 3;
                memcpy(buf, &data[2], len);
                buf[len] = 0;
                preferences.putString("wifi_ssid", (const char *)buf);
                DB_PRINTF("Updated WiFi SSID to: %s\n", buf);
                break;
            case 'k': // Update WiFi key
                len = length - 3;
                memcpy(buf, &data[2], len);
                buf[len] = 0;
                preferences.putString("wifi_key", (const char *)buf);
                DB_PRINTF("Updated WiFi key to: %s\n", buf);
                break;
            case 'm': // WiFi mode (0=AP, 1=use SSID)
                preferences.putUInt("wifi_mode", atoi(&data[2]));
                DB_PRINTF("Updated WiFi mode to (0=access point, 1=connect to SSID): %d\n", atoi(&data[2]));
                break;
#endif                // WEBUI
            case 'n': // Robot name
                len = length - 3;
                memcpy(buf, &data[2], len);
                buf[len] = 0;
                preferences.putString("robot_name", (const char *)buf);
                DB_PRINTF("Updated robot name to: %s\n", buf);
                break;
            }
            break;
        }
        }
    }
}

void setMicroStep(uint8_t uStep)
{
    if (microStep == uStep)
        return;

    microStep = uStep;
    motLeft.microStep = uStep;
    motRight.microStep = uStep;
#ifdef TMC2209                        // Use TMC2209 stepper driver, which uses different microstepping settings
    tmcDriverLeft.microsteps(uStep);  // Set microsteps per step
    tmcDriverRight.microsteps(uStep); // Set microsteps per step
#else
#ifdef STEPPER_DRIVER_TMC2209
    uint8_t ms1, ms2;
    switch (uStep)
    {
    case 8:
        ms1 = 0;
        ms2 = 0;
        break;
    case 16:
        ms1 = 1;
        ms2 = 1;
        break;
    case 32:
        ms1 = 1;
        ms2 = 0;
        break;
    case 64:
        ms1 = 0;
        ms2 = 1;
        break;
    }
    digitalWrite(motLeftUStepPin1, ms1);
    digitalWrite(motLeftUStepPin2, ms2);
    digitalWrite(motRightUStepPin1, ms1);
    digitalWrite(motRightUStepPin2, ms2);

#else
    // input:                     1 2 4 8 16 32
    // uStep table corresponds to 0 1 2 3 4  5  in binary on uStep pins
    // So, we need to take the log2 of input
    uint8_t uStepPow = 0;
    uint8_t uStepCopy = uStep;
    while (uStepCopy >>= 1)
        uStepPow++;

    digitalWrite(motLeftUStepPin1, uStepPow & 0x01);
    digitalWrite(motLeftUStepPin2, uStepPow & 0x02);
    digitalWrite(motRightUStepPin1, uStepPow & 0x01);
    digitalWrite(motRightUStepPin2, uStepPow & 0x02);

#ifdef STEPPER_DRIVER_A4988 // The lookup table for uStepping of the 4988 writes for some reason all three pins high for 1/16th step
    if (uStep == 16)
    {
        digitalWrite(motLeftUStepPin1, 1);
        digitalWrite(motLeftUStepPin2, 1);
        digitalWrite(motRightUStepPin1, 1);
        digitalWrite(motRightUStepPin2, 1);
    }
#endif
#endif // STEPPER_DRIVER_TMC2209
#endif // TMC2209
}
