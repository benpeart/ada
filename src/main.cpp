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
#include <gyro.h>
#include "fsm.h"
#ifdef TMC2209
#include <TMCStepper.h>
#endif // TMC2209
#include "led.h"
#include "wificonnection.h"
#include "webserver.h"
#ifdef INPUT_PS3
#include "ps3.h"
#endif // INPUT_PS3
#ifdef INPUT_XBOX
#include "xbox.h"
#endif // INPUT_XBOX
#include "debug.h"

// TMC2209 Stepper driver
#ifdef TMC2209
#define DRIVER_ADDRESS_LEFT 0b00  // TMC2209 Driver address according to MS1=LOW and MS2=LOW
#define DRIVER_ADDRESS_RIGHT 0b01 // TMC2209 Driver address according to MS1=HIGH and MS2=LOW

#define SERIAL2_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define SERIAL_BAUD_RATE 500000

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

// -- EEPROM
Preferences preferences;
#define PREF_VERSION 1 // if setting structure has been changed, count this number up to delete all settings
#define PREF_NAMESPACE "pref"
#define PREF_KEY_VERSION "ver"

#define BATTERY_VOLTAGE_USB 5       // the voltage when running via USB instead of the battery
#define BATTERY_VOLTAGE_LOW 20      // the voltage we warn the user
#define BATTERY_VOLTAGE_SHUTDOWN 18 // the voltage we shutdown to prevent damaging the battery
#define BATTERY_VOLTAGE_FULL 22.3   // the voltage of a full battery

float BatteryVoltage()
{
    // Measure battery voltage
#ifdef BATTERY_VOLTAGE
    const float R1 = 100000.0;        // 100kΩ
    const float R2 = 10000.0;         // 10kΩ
    const float ADC_MAX = 4095.0;     // 12-bit ADC
    const float V_REF = 3.3;          // Reference voltage
    const float ALPHA = 0.05;         // low pass filter
    static float filteredBattery = 0; // use a low-pass filter to smooth battery readings

    int adcValue = analogRead(PIN_BATTERY_VOLTAGE);
    float voltage = (adcValue / ADC_MAX) * V_REF;
    float batteryVoltage = voltage * (R1 + R2) / R2;

    // take the first and filter the rest
    if (!filteredBattery)
        filteredBattery = batteryVoltage;
    else
        filteredBattery = (ALPHA * batteryVoltage) + ((1 - ALPHA) * filteredBattery);

    // return the battery voltage
    return filteredBattery;
#endif // BATTERY_VOLTAGE
}

// ----- Main code
void setup()
{
    Serial.begin(115200);

    // Init preferences EEPROM, if not done before
    preferences.begin(PREF_NAMESPACE, false); // false = RW-mode
    if (preferences.getUInt(PREF_KEY_VERSION, 0) != PREF_VERSION)
    {
        preferences.clear(); // Remove all preferences under the opened namespace
        preferences.putUInt(PREF_KEY_VERSION, PREF_VERSION);
        DB_PRINTF("EEPROM init complete, all preferences deleted, new pref_version: %d\n", PREF_VERSION);
    }

    // initialize our LED strip and library
    LED_setup();
    LED_set(LED_BATTERY, CRGB::Green);

    // Disable steppers during startup
    pinMode(motEnablePin, OUTPUT);
    digitalWrite(motEnablePin, HIGH); // disable driver in hardware
    LED_set(LED_ENABLED, CRGB::Red);

    // setup micro stepping/serial address pins for output
    pinMode(motLeftUStepPin1, OUTPUT);
    pinMode(motLeftUStepPin2, OUTPUT);
    pinMode(motRightUStepPin1, OUTPUT);
    pinMode(motRightUStepPin2, OUTPUT);

#ifdef TMC2209
    // use TMC2209 pins MS1 and MS2 to set the correct address for Serial control
    digitalWrite(motLeftUStepPin1, LOW);
    digitalWrite(motLeftUStepPin2, LOW);
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
        tmcDriverLeft.mstep_reg_select(true);  // Microstep resolution selected by MRES register instead of MS1 and MS2 pins
        tmcDriverRight.mstep_reg_select(true); // Microstep resolution selected by MRES register instead of MS1 and MS2 pins

        // enable CoolStep to save up to 75% of energy
        tmcDriverLeft.TCOOLTHRS(0xFFFF);  // Set threshold for CoolStep
        tmcDriverRight.TCOOLTHRS(0xFFFF); // Set threshold for CoolStep
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
        tmcDriverLeft.pwm_autoscale(true);    // Enable automatic current scaling for stealthChop
        tmcDriverRight.pwm_autoscale(true);   // Enable automatic current scaling for stealthChop
    }

#endif // TMC2209

    // Gyro setup
    Gyro_setup(dT);

    // Connect to WiFi and setup for OTA updates
    WiFi_setup();

    // setup the AsyncWebServer and WebSocketsServer
    WebServer_setup();

    // setup the balanced controller task
    BalanceController_setup();

// Setup PS3 controller
#ifdef INPUT_PS3
    ps3_setup();
#endif

// Setup Xbox controller
#ifdef INPUT_XBOX
    Xbox_setup();
#endif

    DB_PRINTLN("Booted, ready for driving!");
}

void loop()
{
    // check the battery voltage and if necessary, inform the user
    EVERY_N_MILLISECONDS(5000)
    {
        static float batteryVoltage = BATTERY_VOLTAGE_FULL;
        batteryVoltage = BatteryVoltage();

        // check to see if we're running via USB instead of the battery
        if (batteryVoltage < BATTERY_VOLTAGE_USB)
            batteryVoltage = BATTERY_VOLTAGE_FULL;

        if (batteryVoltage < BATTERY_VOLTAGE_LOW)
        {
            LED_set(LED_BATTERY, CRGB::Yellow);
        }
        if (batteryVoltage <= BATTERY_VOLTAGE_SHUTDOWN)
        {
            DB_PRINTF("Battery voltage is critically low (%.1f). Entering deep sleep mode...\n", batteryVoltage);
            LED_set(LED_BATTERY, CRGB::Red);

            // shut off the motors
            bc.setState(Disabled::GetInstance(), 0, NULL);
            LED_loop();
            esp_deep_sleep_start(); // Enter deep sleep mode
            return;
        }
    }

    WiFi_loop();
    WebServer_loop();
    parseSerial();

    // Handle PS3 controller
#ifdef INPUT_PS3
    Ps3_loop();
#endif

    // Handle Xbox controller
#ifdef INPUT_XBOX
    Xbox_loop();
#endif

    // show any updated LEDs
    LED_loop();
}
