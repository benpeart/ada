#include <Arduino.h>
#include "globals.h"
#ifdef INPUT_XBOX
#include "xbox.h"
#include "debug.h"
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "led.h"
#include <math.h>

#define STEERING_DEADZONE_RADIUS 0.06

// bind to any xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

void onXboxNotify()
{

    xboxController.getReceiveNotificationAt();
    if (xboxController.xboxNotif.btnDirDown)
    {
        remoteControl.speedGain = 0.05;
        remoteControl.steerGain = 0.3;
    }
    if (xboxController.xboxNotif.btnDirLeft)
    {
        remoteControl.speedGain = 0.1;
        remoteControl.steerGain = 0.6;
    }
    if (xboxController.xboxNotif.btnDirUp)
    {
        remoteControl.speedGain = 0.2;
        remoteControl.steerGain = 0.8;
    }
    if (xboxController.xboxNotif.btnDirRight)
    {
        remoteControl.speedGain = 0.25;
        remoteControl.steerGain = 1.0;
    }
    if (xboxController.xboxNotif.btnB)
        remoteControl.selfRight = true;
    if (xboxController.xboxNotif.btnA)
        remoteControl.disableControl = true;
    if (xboxController.xboxNotif.btnRB)
    {
        if (remoteControl.speedOffset < 20.0)
        {
            remoteControl.speedOffset += 0.5;
        }
    }
    if (xboxController.xboxNotif.btnLB)
    {
        if (remoteControl.speedOffset > -20.0)
        {
            remoteControl.speedOffset -= 0.5;
        }
    }
}

void onXboxConnect()
{
    LED_set(LED_CONTROLLER_CONNECTED, CRGB::Green);
    DB_PRINTLN("Bluetooth MAC address: " + xboxController.buildDeviceAddressStr());
    DB_PRINT(xboxController.xboxNotif.toString());
    DB_PRINTLN("Xbox controller connected");
}

void onXboxDisconnect()
{
    LED_set(LED_CONTROLLER_CONNECTED, CRGB::Red);
    DB_PRINTLN("Xbox controller disconnected");
    remoteControl.speed = 0;
    remoteControl.steer = 0;
    remoteControl.speedGain = 1;
    remoteControl.steerGain = 1;
}

void Xbox_setup()
{
    LED_set(LED_CONTROLLER_CONNECTED, CRGB::Red);
    xboxController.begin();
}

void Xbox_loop()
{
    static boolean firstNotification = true;
    xboxController.onLoop();
    if (xboxController.isConnected())
    {
        if (xboxController.isWaitingForFirstNotification())
        {
            //        DB_PRINTLN("waiting for first notification");
        }
        else
        {
            if (firstNotification)
            {
                firstNotification = false;
                onXboxConnect();
            }

            // normalize the controller input to the range of -100 to 100 then apply gain
            float car_speed_forward = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig) * 100 * remoteControl.speedGain + remoteControl.speedOffset;
            float car_speed_reverse = ((float)xboxController.xboxNotif.trigLT / XboxControllerNotificationParser::maxTrig) * 100 * remoteControl.speedGain + remoteControl.speedOffset;

            // subtract the requested reverse speed from the requested forward speed in case both triggers are requesting different values
            remoteControl.speed = -(car_speed_forward - car_speed_reverse);

            // convert the range from 0 <-> maxJoy to -1.0 <-> 1.0
            remoteControl.steer = (float)(xboxController.xboxNotif.joyLHori - (XboxControllerNotificationParser::maxJoy / 2)) / (XboxControllerNotificationParser::maxJoy / 2);

            // if within the dead zone, zero it out
            if (remoteControl.steer > -STEERING_DEADZONE_RADIUS && remoteControl.steer < STEERING_DEADZONE_RADIUS)
                remoteControl.steer = 0;

            // use a response curve to dampen the steering response around center and ramp it up the further you go
            float exponent = 2.0;
            if (remoteControl.steer >= 0)
                remoteControl.steer = pow(remoteControl.steer, exponent);
            else
                remoteControl.steer = -pow(-remoteControl.steer, exponent);
            DB_PRINTF("response curve = %f\n", remoteControl.steer);

            // scale the result according to the d-pad input
            remoteControl.steer = remoteControl.steer * remoteControl.steerGain;

            // now scale it from -1.0 to 1.0 to -100 to 100
            remoteControl.steer *= 100;

            // handle other Xbox inputs
            onXboxNotify();
        }
    }
    else
    {
        if (!firstNotification)
            onXboxDisconnect();
        firstNotification = true;
    }
}
#endif // INPUT_XBOX
