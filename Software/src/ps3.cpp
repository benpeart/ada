#include "globals.h"
#ifdef INPUT_PS3
#include "ps3.h"
#include "debug.h"
#include <Ps3Controller.h>

void onPs3Notify()
{
    if (Ps3.event.button_down.down)
    {
        remoteControl.speedGain = 0.05;
        remoteControl.steerGain = 0.3;
    }
    if (Ps3.event.button_down.left)
    {
        remoteControl.speedGain = 0.1;
        remoteControl.steerGain = 0.6;
    }
    if (Ps3.event.button_down.up)
    {
        remoteControl.speedGain = 0.2;
        remoteControl.steerGain = 0.8;
    }
    if (Ps3.event.button_down.right)
    {
        remoteControl.speedGain = 0.25;
        remoteControl.steerGain = 1.0;
    }
    if (Ps3.event.button_down.circle)
        remoteControl.selfRight = 1;
    if (Ps3.event.button_down.cross)
        remoteControl.disableControl = 1;
    if (Ps3.event.button_down.square)
        remoteControl.override = 1;
    if (Ps3.event.button_down.r1)
    {
        if (remoteControl.speedOffset < 20.0)
        {
            remoteControl.speedOffset += 0.5;
        }
    }
    if (Ps3.event.button_down.r2)
    {
        if (remoteControl.speedOffset > -20.0)
        {
            remoteControl.speedOffset -= 0.5;
        }
    }
}

void onPs3Connect()
{
    digitalWrite(PIN_LED, 1);
    DB_PRINTLN("Bluetooth controller connected");
}

void onPs3Disconnect()
{
    digitalWrite(PIN_LED, 0);
    DB_PRINTLN("Bluetooth controller disconnected");
    remoteControl.speed = 0;
    remoteControl.steer = 0;
    remoteControl.speedGain = 1;
    remoteControl.steerGain = 1;
}
void Ps3_setup()
{
    // Ps3.begin("24:0a:c4:31:3d:86");
    Ps3.attach(onPs3Notify);
    Ps3.attachOnConnect(onPs3Connect);
    Ps3.attachOnDisconnect(onPs3Disconnect);
    Ps3.begin();
    String address = Ps3.getAddress();
    int bt_len = address.length() + 1;
    address.toCharArray(BTaddress, bt_len);
    DB_PRINT("Bluetooth MAC address: ");
    DB_PRINTLN(address);
}

void Ps3_loop()
{
    if (Ps3.isConnected())
    {
        // PS3 input range is -127 ... 127
        remoteControl.speed = -1 * Ps3.data.analog.stick.ry / 1.27 * remoteControl.speedGain + remoteControl.speedOffset;
        remoteControl.steer = Ps3.data.analog.stick.rx / 1.27 * remoteControl.steerGain;
        // Other PS3 inputs are read in a separate interrupt function
    }
}

#endif // INPUT_PS3
