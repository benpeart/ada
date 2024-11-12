#ifndef MYWEBSERVER_H // can't define this as WEBSERVER_H or it conflicts in ESPAsyncWebServer.h
#define MYWEBSERVER_H

#include <Arduino.h>
#include <WebSocketsServer.h>

// ----- Type definitions
typedef union
{
  uint8_t arr[6];
  struct
  {
    uint8_t grp;
    uint8_t cmd;
    union
    {
      float val;
      uint8_t valU8[4];
    };
  } __attribute__((packed));
} cmd;

// Plot settings
typedef struct
{
  boolean enable = 0; // Enable sending data
  uint8_t prescaler = 4;
} plotType;

void sendWifiList(void);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

void WebServer_setup();
void WebServer_loop();

#endif // MYWEBSERVER_H
