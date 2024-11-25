//
// Implement a Finite State Machine to track state changes. The following
// states and conditions are currently supported.
//
//  Current State	Next State	Event/Condition
//  ====================================================================
//  Disabled	Position	Robot is standing
//  Disabled	Self Right	Self right command
//  Driving	    Disabled	Disable command
//  Driving	    Disabled	Robot has fallen
//  Driving	    Disabled	Robot unable to stand
//  Driving	    Position	No speed input from controller for 2 seconds
//  Position	Disabled	Disable command
//  Position	Disabled	Robot has fallen
//  Position	Disabled	Robot unable to stand
//  Position	Driving	    Speed input from controller
//  Self Right	Disabled	Disable command
//  Self Right	Disabled	Robot has fallen
//  Self Right	Disabled	Robot unable to stand
//  Self Right	Position	Robot is standing
//

#include <Arduino.h>
#include "globals.h"
#include "fsm.h"
#include <fastStepper.h>
#include "gyro.h"
#include "led.h"
#include "PID.h"
#include "debug.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

BalanceController bc;

#ifdef FALL_DETECTION
// Threshold for fall detection. If integral of error of angle controller is larger than this value, controller is disabled
#define angleErrorIntegralThreshold 30.0
#define angleErrorIntegralThresholdDuringSelfright angleErrorIntegralThreshold * 3
static float angleErrorIntegral = 0;
#endif // FALL_DETECTION

#define angleEnableThreshold 5.0   // If (absolute) robot angle is below this threshold, enable control
#define angleDisableThreshold 70.0 // If (absolute) robot angle is above this threshold, disable control (robot has fallen down)

// -- PID control
float dT = dT_MICROSECONDS / 1000000.0;

#define PID_ANGLE_MAX 12
PID pidAngle(cPID, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
#define PID_POS_MAX 35
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
#define PID_SPEED_MAX 27
PID pidSpeed(cP, dT, PID_SPEED_MAX, -PID_SPEED_MAX);

// make these global so we can use them in webui.cpp
uint32_t tNowMs;
float pidAngleOutput;
float pidPosOutput;
float pidSpeedOutput;

// Use 'Expodential Smoothing' to improve driving behaviour by preventing abrupt changes in speed or direction
float speedAlphaConstant = 0.1; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
float steerAlphaConstant = 0.1; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
extern float maxStepSpeed;

uint8_t microStep = 0;
float maxStepSpeed = 1500;

// -- Stepper motor
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

fastStepper motLeft(motLeftStepPin, motLeftDirPin, 0, motLeftTimerFunction);
fastStepper motRight(motRightStepPin, motRightDirPin, 1, motRightTimerFunction);

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
        ms1 = LOW;
        ms2 = LOW;
        break;
    case 16:
        ms1 = HIGH;
        ms2 = HIGH;
        break;
    case 32:
        ms1 = HIGH;
        ms2 = LOW;
        break;
    case 64:
        ms1 = LOW;
        ms2 = HIGH;
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
        digitalWrite(motLeftUStepPin1, HIGH);
        digitalWrite(motLeftUStepPin2, HIGH);
        digitalWrite(motRightUStepPin1, HIGH);
        digitalWrite(motRightUStepPin2, HIGH);
    }
#endif
#endif // STEPPER_DRIVER_TMC2209
#endif // TMC2209
}

// Actual balance + controller computations
void balanceLogic(BalanceController *controller, float filterAngle)
{
#ifdef DEBUG_SPINNERS
    const char *spinner = "|/-\\";
    static int i = 0;
    DB_PRINTF("\r%c", spinner[i]);
    i = (i + 1) % sizeof(spinner);
#endif // DEBUG_SPINNERS

    // calculate the correction needed to hit our target setpoint
    pidAngle.input = filterAngle;
    pidAngleOutput = pidAngle.calculate();

    controller->avgMotSpeedSum += pidAngleOutput / 2;
    controller->avgMotSpeedSum = constrain(controller->avgMotSpeedSum, -maxStepSpeed, maxStepSpeed);

    // add in steering input
    motLeft.speed = controller->avgMotSpeedSum + controller->smoothedSteer;
    motRight.speed = controller->avgMotSpeedSum - controller->smoothedSteer;

    // Dynamically switch microstepping to achieve even more insane speeds
#ifdef DYNAMIC_MICROSTEPPING
    float absSpeed = abs(controller->avgMotSpeedSum);

    if (absSpeed > (150 * 32 / microStep) && microStep > 1)
        microStep /= 2;
    if (absSpeed < (130 * 32 / microStep) && microStep < 32)
        microStep *= 2;

    setMicroStep(microStep);
#endif // DYNAMIC_MICROSTEPPING
}

Disabled *Disabled::GetInstance()
{
    static Disabled m_singleton;
    return &m_singleton;
}

void Disabled::enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    // Used ensure we are out of the 'enabled' zone before exiting state Disabled if it was triggered by the controller.
    // Otherwise robot will immediately enter the "Position" mode when it detects we're still standing.
    if (remoteControl && remoteControl->disableControl)
    {
        DB_PRINTLN("Disabled::enter - set needToExitEnableZone");
        needToExitEnableZone = true;
        remoteControl->disableControl = false; // Reset single action bool
    }

    digitalWrite(motEnablePin, HIGH); // disable driver in hardware
    LED_set(LED_ENABLED, CRGB::Red);
    controller->avgMotSpeedSum = 0;
    motLeft.speed = 0;
    motRight.speed = 0;
}

void Disabled::loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
#ifdef DEBUG_SPINNERS
    const char *spinner = ".oOo";
    static int i = 0;
    DB_PRINTF("\r%c", spinner[i]);
    i = (i + 1) % sizeof(spinner);
#endif // DEBUG_SPINNERS

    // clear needToExitEnableZone flag when angle is out of "enable" zone
    if (needToExitEnableZone && (abs(filterAngle) > (angleEnableThreshold + 5)))
    {
        DB_PRINTLN("Disabled::loop - clear needToExitEnableZone");
        needToExitEnableZone = false;
        remoteControl->disableControl = false; // Reset single action bool as it may have been set with a long press
    }

    // detect if we've become vertical and transition to Position
    if (!needToExitEnableZone && (abs(filterAngle) < angleEnableThreshold))
    {
        DB_PRINTLN("Disabled->Position - Robot is standing");
        controller->setState(Position::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Start self-right action when requested by remote control (stops when robot is upright)
    if (remoteControl && remoteControl->selfRight)
    {
        DB_PRINTLN("Disabled->SelfRight - Self right command");
        controller->setState(SelfRight::GetInstance(), filterAngle, remoteControl);
        return;
    }
}

void Disabled::exit(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    // (re-)enable and reset stuff
    digitalWrite(motEnablePin, LOW); // enable driver in hardware
    LED_set(LED_ENABLED, CRGB::Green);
    pidAngle.reset();
    pidPos.reset();
    pidSpeed.reset();
#ifdef FALL_DETECTION
    angleErrorIntegral = 0;                // Reset, otherwise the fall detection will be triggered immediately
#endif // FALL_DETECTION    
    needToExitEnableZone = false;          // Reset in case we self right
    remoteControl->disableControl = false; // Reset single action bool as it may have been set with a long press
}

Driving *Driving::GetInstance()
{
    static Driving m_singleton;
    return &m_singleton;
}

void Driving::enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    pidSpeed.reset();
}

void Driving::loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    // pressed kill-switch
    if (remoteControl && remoteControl->disableControl)
    {
        DB_PRINTLN("Driving->Disabled - Disable command");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Detect robot has fallen if almost horizontal.
    if (abs(filterAngle) > angleDisableThreshold)
    {
        DB_PRINTLN("Driving->Disabled - Robot has fallen");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Detect if robot is unable to stand. Concept: integrate angle controller error over time.
    // If absolute integrated error surpasses threshold, disable controller
#ifdef FALL_DETECTION
    angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
    if (abs(angleErrorIntegral) > angleErrorIntegralThreshold)
    {
        DB_PRINTLN("Driving->Disabled - Robot unable to stand");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }
#endif // FALL_DETECTION

    // Switch to position control if no input is received for a certain amount of time
    if (tNowMs - controller->lastInputTime > 2000)
    {
        DB_PRINTLN("Driving->Position - no driving input");
        controller->setState(Position::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Actual balance computations
    pidSpeed.setpoint = controller->smoothedSpeed;
    pidSpeed.input = -controller->avgMotSpeedSum / 100.0;
    pidSpeedOutput = pidSpeed.calculate();
    pidAngle.setpoint = pidSpeedOutput;
    balanceLogic(controller, filterAngle);
}

Position *Position::GetInstance()
{
    static Position m_singleton;
    return &m_singleton;
}

void Position::enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    motLeft.setStep(0);
    motRight.setStep(0);
    pidPos.reset();
}

void Position::loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    // pressed kill-switch
    if (remoteControl && remoteControl->disableControl)
    {
        DB_PRINTLN("Position->Disabled - Disable command");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Detect robot has fallen if almost horizontal.
    if (abs(filterAngle) > angleDisableThreshold)
    {
        DB_PRINTLN("Driving->Disabled - Robot has fallen");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Detect if robot is unable to stand. Concept: integrate angle controller error over time.
    // If absolute integrated error surpasses threshold, disable controller
#ifdef FALL_DETECTION
    angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
    if (abs(angleErrorIntegral) > angleErrorIntegralThreshold)
    {
        DB_PRINTLN("Position->Disabled - Robot unable to stand");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }
#endif // FALL_DETECTION

    // if we just received speed input
    if (tNowMs == controller->lastInputTime)
    {
        DB_PRINTLN("Position->Driving - Speed input from controller");
        controller->setState(Driving::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Actual balance computations
    int32_t avgMotStep = (motLeft.getStep() + motRight.getStep()) / 2;
    pidPos.setpoint = controller->smoothedSpeed;
    pidPos.input = -((float)avgMotStep) / 1000.0;
    pidPosOutput = pidPos.calculate();
    pidAngle.setpoint = pidPosOutput;
    balanceLogic(controller, filterAngle);
}

SelfRight *SelfRight::GetInstance()
{
    static SelfRight m_singleton;
    return &m_singleton;
}

void SelfRight::enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    remoteControl->selfRight = false; // Reset single action bool
}

void SelfRight::loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    // pressed kill-switch
    if (remoteControl && remoteControl->disableControl)
    {
        DB_PRINTLN("SelfRight->Disabled - Disable command");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Detect if robot is unable to stand. Concept: integrate angle controller error over time.
    // If absolute integrated error surpasses threshold, disable controller
#ifdef FALL_DETECTION    
    angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
    if (abs(angleErrorIntegral) > angleErrorIntegralThresholdDuringSelfright)
    {
        DB_PRINTLN("SelfRight->Disabled - Robot unable to stand");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }
#endif // FALL_DETECTION

    // Detect if robot has successfully self righted
    if (abs(filterAngle) < angleEnableThreshold)
    {
        DB_PRINTLN("SelfRight->Position - Robot is standing");
        controller->setState(Position::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Ignore the throttle/speed and just stand up
    pidAngle.setpoint = 0;
    balanceLogic(controller, filterAngle);
}

const char *stateName(BalanceState *state)
{
#ifdef DEBUG
    if (state == Disabled::GetInstance())
        return "Disabled";
    if (state == Driving::GetInstance())
        return "Driving";
    if (state == Position::GetInstance())
        return "Position";
    if (state == SelfRight::GetInstance())
        return "SelfRight";
#endif
    return "Unknown";
}

void BalanceController::setState(BalanceState *newState, float filterAngle, remoteControlType *remoteControl)
{
    if (m_currentState == newState)
        return;

    DB_PRINTF("BalanceController::setState leaving state %s\n", stateName(m_currentState));
    m_currentState->exit(this, filterAngle, remoteControl); // let the old state clean up after itself
    m_currentState = newState;                              // actually change states
    DB_PRINTF("BalanceController::setState entering state %s\n", stateName(m_currentState));
    m_currentState->enter(this, filterAngle, remoteControl); // let the new state initialize
}

void BalanceController::loop()
{
    static unsigned long tLast = 0;
    unsigned long tNow = micros();

    if (tNow - tLast > dT_MICROSECONDS)
    {
        tLast = tNow;
        float filterAngle = Gyro_ReadSensor(dT);

        // Use 'Expodential Smoothing' to improve driving behaviour by preventing abrupt changes in speed or direction
        // Scale speed down to 20% of the remoteControl.speed value as it is way to fast to control otherwise
        // TODO: may want to dampen steering input as smoothedSpeed increases
        smoothedSpeed = speedAlphaConstant * remoteControl.speed / 5.0 + (1 - speedAlphaConstant) * smoothedSpeed;
        smoothedSteer = steerAlphaConstant * remoteControl.steer + (1 - steerAlphaConstant) * smoothedSteer;

        // save the last speed input time (if it isn't just noise)
        if (abs(smoothedSpeed) >= 0.2)
            lastInputTime = tNowMs;

        // Delegate the task of determining the next state to the current state
        m_currentState->loop(this, filterAngle, &remoteControl);

        motLeft.update();
        motRight.update();
    }
}

void BalanceController_task(void *pvParameters)
{
    while (true)
    {
        // run the balancing logic
        bc.loop();
    }
}

void BalanceController_setup()
{
    // initialize all our PIDs with default values
    pidAngle.setParameters(0.65, 1.0, 0.075, 15);
    pidPos.setParameters(1, 0, 1.2, 50);
    pidSpeed.setParameters(6, 5, 0, 20);

    // setup stepper motors
    setMicroStep(16);
    motLeft.init();
    motRight.init();

    xTaskCreate(BalanceController_task, "BalanceController_task", 2048, NULL, 12, NULL);
}