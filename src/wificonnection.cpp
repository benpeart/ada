#include <Arduino.h>
#include "globals.h"
#ifdef WIFI_CONNECTION
#include <WiFi.h>
#ifdef MDNS
#include <ESPmDNS.h>
#endif // MDNS
#include <ArduinoOTA.h>
#include "wificonnection.h"
#include "webserver.h"
#include "debug.h"

char robotName[32] = "ada"; // -- Used as WiFi network name

void connectToWifi()
{
    // Connect to Wifi and setup AP if known Wifi network cannot be found
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
}

void WiFi_setup()
{
    // Read robot name
    preferences.getString("robot_name", robotName, sizeof(robotName));
    DB_PRINTLN(robotName);
    WiFi.setHostname(robotName);

    // Connect to Wifi and setup AP if known Wifi network cannot be found
    connectToWifi();

    // Setup for OTA updates
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
                        // TODO: is this needed?
                        // SPIFFS.end();
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

    // Start MDNS server
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

    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws", "tcp", 81);
#endif // MDNS
}

void WiFi_loop()
{
    static uint8_t k = 0;

    if ((WiFi.status() != WL_CONNECTED))
    {
        DB_PRINTLN(F("\nWiFi lost. Attempting to reconnect"));
        connectToWifi();
    }

    // check for OTA updates
    ArduinoOTA.handle();
}

#else

void WiFi_setup() {};
void WiFi_loop() {};

#endif // WIFI_CONNECTION
