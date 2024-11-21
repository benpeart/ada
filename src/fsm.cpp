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
#include "gyro.h"
#include "led.h"
#include "debug.h"

// Threshold for fall detection. If integral of error of angle controller is larger than this value, controller is disabled
#define angleErrorIntegralThreshold 30.0
#define angleErrorIntegralThresholdDuringSelfright angleErrorIntegralThreshold * 3
static float angleErrorIntegral = 0;

#define angleEnableThreshold 5.0   // If (absolute) robot angle is below this threshold, enable control
#define angleDisableThreshold 70.0 // If (absolute) robot angle is above this threshold, disable control (robot has fallen down)

// TODO: move these
#define dT_MICROSECONDS 5000
extern float dT;
extern PID pidAngle;
extern PID pidPos;
extern PID pidSpeed;
extern float pidAngleOutput;
extern float pidPosOutput;
extern float pidSpeedOutput;
extern float speedAlphaConstant; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
extern float steerAlphaConstant; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
extern float maxStepSpeed;

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
    float avgMotSpeed = controller->avgMotSpeedSum;

    // add in steering input
    motLeft.speed = avgMotSpeed + controller->smoothedSteer;
    motRight.speed = avgMotSpeed - controller->smoothedSteer;

    // Dynamically switch microstepping to achieve even more insane speeds
#ifdef DYNAMIC_MICROSTEPPING
    float absSpeed = abs(avgMotSpeed);

    if (absSpeed > (150 * 32 / microStep) && microStep > 1)
        microStep /= 2;
    if (absSpeed < (130 * 32 / microStep) && microStep < 32)
        microStep *= 2;

    setMicroStep(microStep);
#endif // DYNAMIC_MICROSTEPPING
}

Disabled::Disabled()
{
    needToExitEnableZone = false;
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
        needToExitEnableZone = true;
        remoteControl->disableControl = false; // Reset single action bool
    }

    digitalWrite(motEnablePin, HIGH); // disable driver in hardware
    LED_set(LED_ENABLED, CRGB::Red);
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

    // Reset needToExitEnableZone flag when angle is out of "enable" zone
    if (needToExitEnableZone && (abs(filterAngle) > angleEnableThreshold + 5))
    {
        DB_PRINTLN("Disabled - clear needToExitEnableZone");
        needToExitEnableZone = false;
    }

    // detect if we've become vertical and transition to Position
    if (!needToExitEnableZone && abs(filterAngle) < angleEnableThreshold)
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

void Disabled::exit(BalanceController *controller)
{
    // (re-)enable and reset stuff
    digitalWrite(motEnablePin, LOW); // enable driver in hardware
    LED_set(LED_ENABLED, CRGB::Green);
    controller->avgMotSpeedSum = 0;
    motLeft.setStep(0);
    motRight.setStep(0);
    pidAngle.reset();
    pidPos.reset();
    pidSpeed.reset();
    angleErrorIntegral = 0; // Reset, otherwise the fall detection will be triggered immediately
}

Driving *Driving::GetInstance()
{
    static Driving m_singleton;
    return &m_singleton;
}

void Driving::enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl)
{
    motLeft.setStep(0);
    motRight.setStep(0);
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
    angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
    if (abs(angleErrorIntegral) > angleErrorIntegralThreshold)
    {
        DB_PRINTLN("Driving->Disabled - Robot unable to stand");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }

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
    angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
    if (abs(angleErrorIntegral) > angleErrorIntegralThreshold)
    {
        DB_PRINTLN("Position->Disabled - Robot unable to stand");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }

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
    angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
    if (abs(angleErrorIntegral) > angleErrorIntegralThresholdDuringSelfright)
    {
        DB_PRINTLN("SelfRight->Disabled - Robot unable to stand");
        controller->setState(Disabled::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Detect if robot has successfully self righted
    if (abs(filterAngle) < angleEnableThreshold)
    {
        DB_PRINTLN("SelfRight->Position - Robot is standing");
        controller->setState(Position::GetInstance(), filterAngle, remoteControl);
        return;
    }

    // Actual balance + controller computations
    pidSpeed.setpoint = controller->smoothedSpeed;
    pidSpeed.input = -controller->avgMotSpeedSum / 100.0;
    pidSpeedOutput = pidSpeed.calculate();
    pidAngle.setpoint = pidSpeedOutput;
    balanceLogic(controller, filterAngle);
}

BalanceController::BalanceController()
{
    // start off in the disabled state
    smoothedSteer = 0;
    smoothedSpeed = 0;
    lastInputTime = 0;
    avgMotSpeedSum = 0;
    m_currentState = Disabled::GetInstance();
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
    m_currentState->exit(this);                              // let the old state clean up after itself
    m_currentState = newState;                               // actually change states
    m_currentState->enter(this, filterAngle, remoteControl); // let the new state initialize
    DB_PRINTF("BalanceController::setState entering state %s\n", stateName(m_currentState));
}

void BalanceController::loop(float filterAngle, remoteControlType *remoteControl)
{
    static unsigned long tLast = 0;
    unsigned long tNow = micros();

    if (tNow - tLast > dT_MICROSECONDS)
    {
        tLast = tNow;
        Gyro_ReadSensor();

        // Use 'Expodential Smoothing' to improve driving behaviour by preventing abrupt changes in speed or direction
        // Scale speed down to 20% of the remoteControl.speed value as it is way to fast to control otherwise
        // TODO: may want to dampen steering input as smoothedSpeed increases
        smoothedSpeed = speedAlphaConstant * remoteControl->speed / 5.0 + (1 - speedAlphaConstant) * smoothedSpeed;
        smoothedSteer = steerAlphaConstant * remoteControl->steer + (1 - steerAlphaConstant) * smoothedSteer;

        // save the last speed input time (if it isn't just noise)
        if (abs(smoothedSpeed) >= 0.2)
            lastInputTime = tNowMs;

        // Delegate the task of determining the next state to the current state
        m_currentState->loop(this, filterAngle, remoteControl);

        motLeft.update();
        motRight.update();
    }
}
