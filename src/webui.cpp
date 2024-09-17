#include <WiFi.h>
#ifdef MDNS
#include <ESPmDNS.h>
#endif // MDNS
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
#ifdef SPIFFSEDITOR
#include <SPIFFSEditor.h>
#endif // SPIFFSEDITOR
#include "webui.h"
#include "debug.h"
#include "globals.h"

// -- Web server
char robotName[32] = "ada"; // -- Used as WiFi network name
AsyncWebServer httpServer(80);
WebSocketsServer wsServer = WebSocketsServer(81);

plotType plot;

void sendConfigurationData(uint8_t num);

bool WebUI_setup()
{
    // Read robot name
    preferences.getString("robot_name", robotName, sizeof(robotName));
    DB_PRINTLN(robotName);

    // Connect to Wifi and setup OTA if known Wifi network cannot be found
    WiFi.setHostname(robotName);
    boolean wifiConnected = 0;
    if (preferences.getUInt("wifi_mode", 1) == 1)
    {
        char ssid[32] = "IOT";
        char key[63] = "";
        preferences.getString("wifi_ssid", ssid, sizeof(ssid));
        preferences.getString("wifi_key", key, sizeof(key));
        DB_PRINTF("Connecting to '%s'\n", ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, key);
        if (!(WiFi.waitForConnectResult() != WL_CONNECTED))
        {
            DB_PRINT("Connected to WiFi with IP address: ");
            DB_PRINTLN(WiFi.localIP());
            wifiConnected = 1;
        }
        else
        {
            DB_PRINTLN("Could not connect to known WiFi network");
        }
    }
    if (!wifiConnected)
    {
        DB_PRINTLN("Starting AP...");
        WiFi.mode(WIFI_AP_STA);
        // WiFi.softAPConfig(apIP, apIP, IPAddress(192,168,178,24));
        WiFi.softAP(robotName);
        DB_PRINTF("AP named '%s' started, IP address: %s\n", WiFi.softAPSSID(), WiFi.softAPIP().toString());
    }

    ArduinoOTA.setHostname(robotName);
    ArduinoOTA
        .onStart([]()
                 {
                    String type;
                    if (ArduinoOTA.getCommand() == U_FLASH)
                    {
                        type = "sketch";
                    } else { // U_SPIFFS
                        type = "filesystem";
                        SPIFFS.end();
                    }
                    DB_PRINTLN("Start updating " + type); })
        .onEnd([]()
               { DB_PRINTLN("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { DB_PRINTF("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
                    DB_PRINTF("Error[%u]: ", error);
                    if (error == OTA_AUTH_ERROR) DB_PRINTLN("Auth Failed");
                    else if (error == OTA_BEGIN_ERROR) DB_PRINTLN("Begin Failed");
                    else if (error == OTA_CONNECT_ERROR) DB_PRINTLN("Connect Failed");
                    else if (error == OTA_RECEIVE_ERROR) DB_PRINTLN("Receive Failed");
                    else if (error == OTA_END_ERROR) DB_PRINTLN("End Failed"); });

    ArduinoOTA.begin();
    DB_PRINTLN("Ready for OTA updates");

    // Start DNS server
#ifdef MDNS
    if (MDNS.begin(robotName))
    {
        DB_PRINT("MDNS responder started, name: ");
        DB_PRINTLN(robotName);
    }
    else
    {
        DB_PRINTLN("Could not start MDNS responder");
    }
#endif // MDNS

    // SPIFFS setup
    if (!SPIFFS.begin(true))
    {
        DB_PRINTLN("SPIFFS mount failed");
        return false;
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

#ifdef MDNS
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws", "tcp", 81);
#endif // MDNS

    return true;
}

void WebUI_loop()
{
    static uint8_t k = 0;

    // Measure battery voltage, and send to connected client(s), if any
#ifdef BATTERY_VOLTAGE
    float newBatteryVoltage = 0; // analogRead(PIN_BATTERY_VOLTAGE);
    uint32_t reading = adc1_get_raw(ADC_CHANNEL_BATTERY_VOLTAGE);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(reading, &adc_chars);
    avgBatteryVoltage = avgBatteryVoltage * BATTERY_VOLTAGE_FILTER_COEFFICIENT + (voltage / 1000.0) * BATTERY_VOLTAGE_SCALING_FACTOR * (1 - BATTERY_VOLTAGE_FILTER_COEFFICIENT);

    // Send battery voltage readout periodically to web page, if any clients are connected
    static unsigned long tLastBattery;
    if (tNowMs - tLastBattery > 5000)
    {
        if (wsServer.connectedClients(0) > 0)
        {
            char wBuf[10];

            sprintf(wBuf, "b%.1f", avgBatteryVoltage);
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

    // Run other tasks
    ArduinoOTA.handle();
    wsServer.loop();
}

void sendWifiList(void)
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
                    remoteControl.selfRight = 1;
                    break;
                case 3:
                    remoteControl.disableControl = 1;
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

void sendConfigurationData(uint8_t num)
{
    // send message to client
    char wBuf[65];
    char buf[63];
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
    sprintf(wBuf, "h%.4f", speedFilterConstant);
    wsServer.sendTXT(num, wBuf);
    sprintf(wBuf, "i%.4f", steerFilterConstant);
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
