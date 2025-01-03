#include "globals.h"
#include "webserver.h"
#include "debug.h"
#include "fsm.h"
#include "gyro.h"
#ifdef WEBSERVER
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
#ifdef SPIFFSEDITOR
#include <SPIFFSEditor.h>
#endif // SPIFFSEDITOR
#include "wificonnection.h"

// -- Web server
AsyncWebServer httpServer(80);
WebSocketsServer wsServer = WebSocketsServer(81);

plotType plot;

void sendWifiList()
{
    char wBuf[200];
    uint8_t n;
    uint16_t pos = 2;

    wBuf[0] = 'w';
    wBuf[1] = 'l';

    DB_PRINTLN("Scan started");
    n = WiFi.scanNetworks();

    if (n > 5)
        n = 5; // Limit to first 5 SSIDs

    // Make concatenated list, separated with commas
    for (uint8_t i = 0; i < n; i++)
    {
        pos += sprintf(wBuf + pos, "%s,", WiFi.SSID(i).c_str());
    }
    wBuf[pos - 1] = 0;

    DB_PRINTLN(wBuf);
    wsServer.sendTXT(0, wBuf);
}

#endif // WEBSERVER

void parseCommand(char *data, uint8_t length)
{
#ifdef SERIALINPUT
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
        case 'u':
            setMicroStep(atoi(data + 1));
            break;
        case 'g':
            gyroGain = atof(data + 1);
            break;
#ifdef WEBSERVER
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
#endif // WEBSERVER
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
#ifdef WEBSERVER
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
#endif                // WEBSERVER
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
#endif // SERIALINPUT
}

void parseSerial()
{
#ifdef SERIALINPUT
    static char serialBuf[64];
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
#endif // SERIALINPUT
}

#ifdef WEBSERVER

void sendConfigurationData(uint8_t num)
{
    // send message to client
    char wBuf[65] = "";
    char buf[63] = "";

    sprintf(wBuf, "c%dp%.4f", 1, pidAngle.K);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%di%.4f", 1, pidAngle.Ti);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dd%.4f", 1, pidAngle.Td);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dn%.4f", 1, pidAngle.N);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dr%.4f", 1, pidAngle.R);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dm%.4f", 1, pidAngle.maxOutput);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%do%.4f", 1, -pidAngle.minOutput);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dp%.4f", 2, pidPos.K);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%di%.4f", 2, pidPos.Ti);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dd%.4f", 2, pidPos.Td);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dn%.4f", 2, pidPos.N);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dr%.4f", 2, pidPos.R);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dm%.4f", 2, pidPos.maxOutput);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%do%.4f", 2, -pidPos.minOutput);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dp%.4f", 3, pidSpeed.K);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%di%.4f", 3, pidSpeed.Ti);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dd%.4f", 3, pidSpeed.Td);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dn%.4f", 3, pidSpeed.N);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dr%.4f", 3, pidSpeed.R);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%dm%.4f", 3, pidSpeed.maxOutput);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "c%do%.4f", 3, -pidSpeed.minOutput);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "h%.4f", speedAlphaConstant);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "i%.4f", steerAlphaConstant);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "j%.4f", gyroGain);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "n%.4f", gyroFilterConstant);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "l%.4f", maxStepSpeed);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "wm%d", preferences.getUInt("wifi_mode", 0)); // 0=AP, 1=Client
    wsServer.sendTXT(num, wBuf);
    preferences.getString("wifi_ssid", buf, sizeof(buf));
    sprintf(wBuf, "ws%s", buf);
    wsServer.sendTXT(num, wBuf);
    preferences.getString("wifi_key", buf, sizeof(buf));
    sprintf(wBuf, "wk%s", buf);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "wn%s", robotName);
    wsServer.sendTXT(num, wBuf);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        DB_PRINTF("[%u] Disconnected!\n", num);
        break;
    case WStype_CONNECTED:
    {
        IPAddress ip = wsServer.remoteIP(num);
        DB_PRINTF("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        sendConfigurationData(num);
    }
    break;
    case WStype_TEXT:
        DB_PRINTF("[%u] get Text: %s\n", num, payload);
        parseCommand((char *)payload, length);
        break;
    case WStype_BIN:
    {
        // DB_PRINTF("[%u] get binary length: %u\n", num, length);
        if (length == 6)
        {
            cmd c;
            memcpy(c.arr, payload, 6);
            // Serial << "Binary: " << c.grp << "\t" << c.cmd << "\t" << c.val << "\t" << sizeof(cmd) << endl;
            if (c.grp == 100)
            {
                switch (c.cmd)
                {
                case 0:
                    remoteControl.speed = c.val;
                    break;
                case 1:
                    remoteControl.steer = c.val;
                    break;
                case 2:
                    remoteControl.selfRight = true;
                    break;
                case 3:
                    remoteControl.disableControl = true;
                    break;
                }
                // if (c.cmd==0) {
                //   remoteControl.speed = c.val;
                // } else if (c.cmd==1) {
                //   remoteControl.steer = c.val;
                // }
            }
        }

        break;
    }
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
        break;
    }
}
#endif // WEBSERVER

void WebServer_setup()
{
#ifdef WEBSERVER
    // SPIFFS setup
    if (!SPIFFS.begin(true))
    {
        DB_PRINTLN("SPIFFS mount failed");
        return;
    }
    else
    {
        DB_PRINTLN("SPIFFS mount success");
    }

    // setup the Async Web Server
    httpServer.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
    httpServer.onNotFound([](AsyncWebServerRequest *request)
                          { request->send(404, "text/plain", "FileNotFound"); });
#ifdef SPIFFSEDITOR
    httpServer.addHandler(new SPIFFSEditor(SPIFFS));
#endif // SPIFFSEDITOR
    httpServer.begin();

    wsServer.onEvent(webSocketEvent);
    wsServer.begin();
#endif // WEBSERVER
}

void WebServer_loop()
{
#ifdef WEBSERVER
    static unsigned long tLast = 0;
    unsigned long tNow = micros();
    static uint8_t k = 0;

    // run the winsock server
    wsServer.loop();

    // update the web page at the same rate we read the gyro so our plot.prescaler logic still works
    if (tNow - tLast > dT_MICROSECONDS)
        return;
    tLast = tNow;

    // Measure battery voltage, and send to connected client(s), if any
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

    // Send battery voltage readout periodically to web page, if any clients are connected
    static unsigned long tLastBattery;
    if (tNowMs - tLastBattery > 5000)
    {
        if (wsServer.connectedClients(0) > 0)
        {
            char wBuf[10];

            sprintf(wBuf, "b%.1f", filteredBattery);
            wsServer.broadcastTXT(wBuf);
        }
        tLastBattery = tNowMs;
    }
#endif // BATTERY_VOLTAGE
    if (k == plot.prescaler)
    {
        k = 0;

        //        DB_PRINTF("WebUI_Loop plot.enable = %d, wsServer.connectedClients =  %d\n", plot.enable, wsServer.connectedClients());
        if (plot.enable && wsServer.connectedClients() > 0)
        {
            union
            {
                struct
                {
                    uint8_t cmd;
                    uint8_t fill1;
                    uint8_t fill2;
                    uint8_t fill3;
                    float f[14];
                };
                uint8_t b[56];
            } plotData;

            plotData.cmd = 255;
            plotData.f[0] = micros() / 1000000.0;
            plotData.f[1] = accAngle;
            plotData.f[2] = filterAngle;
            plotData.f[3] = pidAngle.setpoint;
            plotData.f[4] = pidAngle.input;
            plotData.f[5] = pidAngleOutput;
            plotData.f[6] = pidPos.setpoint;
            plotData.f[7] = pidPos.input;
            plotData.f[8] = pidPosOutput;
            plotData.f[9] = pidSpeed.setpoint;
            plotData.f[10] = pidSpeed.input;
            plotData.f[11] = pidSpeedOutput;
            plotData.f[12] = motLeft.speed;
            plotData.f[13] = motRight.speed;
            wsServer.sendBIN(0, plotData.b, sizeof(plotData.b));
        }
    }
    k++;
#endif // WEBSERVER
}
