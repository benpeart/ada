#ifndef FSM_H
#define FSM_H

#include <fastStepper.h>
#include "PID.h"

//
// Implement a Finite State Machine to track balancing state changes. The following
// states and conditions are currently supported.
//
//  Current State	Next State	Event/Condition
//  ====================================================================
//  Disabled	    Position	Robot is standing
//  Disabled	    Self Right	Self right command
//  Driving	        Disabled	Disable command
//  Driving	        Disabled	Robot has fallen
//  Driving	        Disabled	Robot unable to stand
//  Driving	        Position	No speed input from controller for 2 seconds
//  Position	    Disabled	Disable command
//  Position	    Disabled	Robot has fallen
//  Position	    Disabled	Robot unable to stand
//  Position	    Driving	    Speed input from controller
//  Self Right	    Disabled	Disable command
//  Self Right	    Disabled	Robot unable to stand
//  Self Right	    Position	Robot is standing
//

class BalanceController;

class BalanceState
{
public:
    virtual void enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl) = 0;
    virtual void loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl) = 0;
    virtual void exit(BalanceController *controller, float filterAngle, remoteControlType *remoteControl) = 0;
};

class Disabled : public BalanceState
{
    // Used ensure we are out of the 'enabled' zone before exiting state Disabled if it was triggered by the controller.
    // Otherwise robot will immediately enter the "Position" mode when it detects we're still standing.
    bool needToExitEnableZone = false;

public:
    static Disabled *GetInstance();
    void enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
    void loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
    void exit(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
};

class Driving : public BalanceState
{
public:
    static Driving *GetInstance();
    void enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
    void loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
    void exit(BalanceController *controller, float filterAngle, remoteControlType *remoteControl) {};
};

class Position : public BalanceState
{
public:
    static Position *GetInstance();
    void enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
    void loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
    void exit(BalanceController *controller, float filterAngle, remoteControlType *remoteControl) {};
};

class SelfRight : public BalanceState
{
public:
    static SelfRight *GetInstance();
    void enter(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
    void loop(BalanceController *controller, float filterAngle, remoteControlType *remoteControl);
    void exit(BalanceController *controller, float filterAngle, remoteControlType *remoteControl) {};
};

class BalanceController
{
private:
    // start off in the disabled state
    BalanceState *m_currentState = Disabled::GetInstance();

public:
    float smoothedSteer = 0;
    float smoothedSpeed = 0;
    float avgMotSpeedSum = 0;
    uint32_t lastInputTime = 0;

    void setState(BalanceState *newState, float filterAngle, remoteControlType *remoteControl);
    void loop();
};

extern BalanceController bc;
void BalanceController_setup();

// these are all needed so we can plot them in the WebUI
#define dT_MICROSECONDS 5000
extern float dT;
extern float gyroFilterConstant;
extern float gyroGain;

extern uint32_t tNowMs;
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

extern fastStepper motLeft;
extern fastStepper motRight;
void setMicroStep(uint8_t uStep);

#endif // FSM_H
