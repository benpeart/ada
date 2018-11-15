/*
A high speed balancing robot, running on an ESP32.

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
#include <FlySkyIBus.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Streaming.h>
#include <MPU6050.h>
#include <EEPROM.h>
#include <PID.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <fastStepper.h>

// ----- Type definitions
typedef union {
  struct {
    float val; // Float (4 bytes) comes first, as otherwise padding will be applied
    uint8_t cmd;
    uint8_t checksum;
  };
  uint8_t array[6];
} command;

// Plot settings
struct {
  boolean enable = 0; // Enable sending data
  uint8_t prescaler = 4;
} plot;

#define FORMAT_SPIFFS_IF_FAILED true

// ----- Function prototypes
void sendWifiList(void);
void parseSerial();
void parseCommand(char* data, uint8_t length);
void calculateGyroOffset(uint8_t nSample);
void readSensor();
void initSensor(uint8_t n);
void setMicroStep(uint8_t uStep);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();

// ----- Definitions and variables
// -- Web server
const char* http_username = "admin";
const char* http_password = "admin";
AsyncWebServer httpServer(80);
WebSocketsServer wsServer = WebSocketsServer(81);

// -- EEPROM
#define EEPROM_SIZE 256
#define EEPROM_ADR_INIT 0
#define EEPROM_ADR_GYRO_OFFSET 50
#define EEPROM_ADR_ANGLE_OFFSET 60
#define EEPROM_ADR_WIFI_SSID 70
#define EEPROM_ADR_WIFI_KEY 100
#define EEPROM_ADR_WIFI_MODE 69

// -- Stepper motors
#define motEnablePin 27
#define motUStepPin1 14
#define motUStepPin2 12
#define motUStepPin3 13

fastStepper motLeft(5, 4, 0, motLeftTimerFunction);
fastStepper motRight(2, 15, 1, motRightTimerFunction);

uint8_t microStep = 32;
uint8_t motorCurrent = 100;
float maxStepSpeed = 3000;

// -- PID control
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0

#define PID_ANGLE_MAX 20
PID pidAngle(cPD, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
#define PID_POS_MAX 35
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
PID pidSpeed(cP, dT, PID_POS_MAX, -PID_POS_MAX);

uint8_t controlMode = 1; // 0 = only angle, 1 = angle+position, 2 = angle+speed

// -- IMU
MPU6050 imu;

#define GYRO_SENSITIVITY 65.5

int16_t gyroOffset[3];
float accAngle = 0;
float filterAngle = 0;
float angleOffset = 2.0;
float gyroFilterConstant = 0.996;
float gyroGain = 1.1;

// -- Others
#define ledPin 2
#define motorCurrentPin 25
#define battVoltagePin 34

float steerFilterConstant = 0.7;
float speedFilterConstant = 0.9;

// -- WiFi
const char host[] = "balancingrobot";

// ----- Interrupt functions -----
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR motLeftTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setMotorCurrent() {
  dacWrite(motorCurrentPin, motorCurrent);
}

// ----- Main code
void setup() {

  Serial.begin(115200);
  IBus.begin(Serial2);
  EEPROM.begin(EEPROM_SIZE);

  pinMode(motEnablePin, OUTPUT);
  pinMode(motUStepPin1, OUTPUT);
  pinMode(motUStepPin2, OUTPUT);
  pinMode(motUStepPin3, OUTPUT);
  digitalWrite(motEnablePin, 1); // Disable steppers during startup
  setMicroStep(microStep);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);

  motLeft.init();
  motRight.init();
  motLeft.microStep = microStep;
  motRight.microStep = microStep;

  // SPIFFS setup
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    Serial.println("SPIFFS mount failed");
    return;
  } else {
    Serial.println("SPIFFS mount success");
  }

  // Gyro setup
  delay(200);
  Wire.begin(21,22,400000);
  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  // Calculate and store gyro offsets
  delay(50);

  // Init EEPROM, if not done before
  if (EEPROM.read(EEPROM_ADR_INIT) != 123) {
    EEPROM.write(EEPROM_ADR_INIT, 123);
    for (uint16_t i=1; i<EEPROM_SIZE; i++) {
      EEPROM.write(i, 0);
    }
    Serial.println("EEPROM init complete");
  }

  // Read gyro offsets
  Serial << "Gyro calibration values: ";
  for (uint8_t i=0; i<3; i++) {
    gyroOffset[i] = EEPROM.readShort(EEPROM_ADR_GYRO_OFFSET + i*2);
    Serial << gyroOffset[i] << "\t";
  }
  Serial << endl;

  // Read angle offset
  angleOffset = EEPROM.readFloat(EEPROM_ADR_ANGLE_OFFSET);

  // Perform initial gyro measurements
  initSensor(50);

  // Connect to Wifi and setup OTA if known Wifi network cannot be found
  boolean wifiConnected = 0;
  if (EEPROM.read(EEPROM_ADR_WIFI_MODE)==1) {
    char ssid[30];
    char key[30];
    EEPROM.readString(EEPROM_ADR_WIFI_SSID, ssid, 30);
    EEPROM.readString(EEPROM_ADR_WIFI_KEY, key, 30);
    Serial << "Connecting to " << ssid << endl;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, key);
    if (!(WiFi.waitForConnectResult() != WL_CONNECTED)) {
      Serial.print("Connected to WiFi with IP address: ");
      Serial.println(WiFi.localIP());
      wifiConnected = 1;
    } else {
      Serial.println("Could not connect to known WiFi network");
    }
  }
  if (!wifiConnected) {
    Serial.println("Starting AP...");
    WiFi.mode(WIFI_AP_STA);
    // WiFi.softAPConfig(apIP, apIP, IPAddress(192,168,178,24));
    WiFi.softAP("balancingRobot", "turboturbo");
    Serial.print("AP started with IP address: ");
    Serial.println(WiFi.softAPIP());
  }

    ArduinoOTA.setHostname(host);
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  // Start DNS server
  if (MDNS.begin(host)) {
    Serial.print("MDNS responder started, name: ");
    Serial.println(host);
  }

  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Loading index.htm");
    request->send(SPIFFS, "/index.htm");
  });

  httpServer.serveStatic("/", SPIFFS, "/");
  httpServer.onNotFound([](AsyncWebServerRequest *request){
      request->send(404, "text/plain", "FileNotFound");
  });

  httpServer.addHandler(new SPIFFSEditor(SPIFFS,http_username,http_password));
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(webSocketEvent);

  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);

  // Make some funny sounds
  // for (uint8_t i=0; i<150; i++) {
  //   motRight.speed = 500 + i*10;
  //   updateStepper(&motRight);
  //   delay(5);
  // }

  dacWrite(motorCurrentPin, motorCurrent);
  pidAngle.setParameters(0.65,0,0.075,15);
  pidAngle.setpoint = 0;

  pidPos.setParameters(1,0,1.2,20);
  pidPos.setpoint = 0;

  pidSpeed.setParameters(6,5,0,20);
  pidSpeed.setpoint = 0;

  Serial.println("Ready");
}



void loop() {
  static unsigned long tLast = 0;
  float pidAngleOutput = 0;
  float avgMotSpeed;
  float steer = 0;
  static float avgSteer;
  static float avgSpeed;
  static boolean enableControl = 0;
  static float avgMotSpeedSum = 0;
  int32_t avgMotStep;
  float pidPosOutput = 0, pidSpeedOutput = 0;
  static uint8_t k = 0;
  static float avgBatteryVoltage = 0;
  static uint32_t lastInputTime = 0;
  uint32_t tNowMs;
  float absSpeed = 0;

  unsigned long tNow = micros();
  tNowMs = millis();

  if (tNow-tLast > dT_MICROSECONDS) {
    readSensor();
    if (enableControl) {
      // Read receiver inputs
      if (IBus.readChannel(0)>0) { // Check if receiver is active
        avgSpeed = speedFilterConstant*avgSpeed + (1-speedFilterConstant)*(((float) IBus.readChannel(1)-1500)/50.0);
        avgSteer = steerFilterConstant*avgSteer + (1-steerFilterConstant)*(((float) IBus.readChannel(0)-1500)/4.0);
        // uint8_t lastControlMode = controlMode;
        // controlMode = (2000-IBus.readChannel(5))/450;

        if (abs(avgSpeed)<0.2) {
          // speedInput = 0;
        } else {
          lastInputTime = tNowMs;
          if (controlMode==1) {
            controlMode = 2;
            motLeft.setStep(0);
            motRight.setStep(0);
            pidSpeed.resetDTerm();
          }
        }

        if (abs(avgSteer)>1) {
          steer = avgSteer * (1 - abs(avgSpeed)/150.0);
        } else {
          steer = 0;
        }

      }

      if (tNowMs-lastInputTime>2000 && controlMode == 2) {
        controlMode = 1;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.resetDTerm();
      }

      if (controlMode == 0) {
        pidAngle.setpoint = avgSpeed*2;
      } else if (controlMode == 1) {
        avgMotStep = (motLeft.getStep() + motRight.getStep())/2;
        pidPos.setpoint = avgSpeed;
        pidPos.input = -((float) avgMotStep) / 1000.0;
        pidPosOutput = pidPos.calculate();
        pidAngle.setpoint = pidPosOutput;
      } else if (controlMode == 2) {
        pidSpeed.setpoint = avgSpeed;
        pidSpeed.input = -avgMotSpeedSum/100.0;
        pidSpeedOutput = pidSpeed.calculate();
        pidAngle.setpoint = pidSpeedOutput;
      }

      pidAngle.input = filterAngle;
      pidAngleOutput = pidAngle.calculate();

      avgMotSpeedSum += pidAngleOutput/2;
      if (avgMotSpeedSum>maxStepSpeed) {
        avgMotSpeedSum  = maxStepSpeed;
      } else if (avgMotSpeedSum<-maxStepSpeed) {
        avgMotSpeedSum  = -maxStepSpeed;
      }
      avgMotSpeed = avgMotSpeedSum;
      motLeft.speed = avgMotSpeed + steer;
      motRight.speed = avgMotSpeed - steer;

      // Switch microstepping
      absSpeed = abs(avgMotSpeed);
      uint8_t lastMicroStep = microStep;

      if (absSpeed > (150 * 32 / microStep) && microStep > 1) microStep /= 2;
      if (absSpeed < (130 * 32 / microStep) && microStep < 32) microStep *= 2;

      if (microStep!=lastMicroStep) {
        motLeft.microStep = microStep;
        motRight.microStep = microStep;
        setMicroStep(microStep);
      }

      // Disable control if robot is almost horizontal. Re-enable if upright.
      if (abs(filterAngle)>70) {
        enableControl = 0;
        motLeft.speed = 0;
        motRight.speed = 0;
        digitalWrite(motEnablePin, 1); // Inverted action on enable pin
      }
    } else {
      if (abs(filterAngle)<0.5) { // (re-)enable and reset stuff
        enableControl = 1;
        controlMode = 1;
        avgMotSpeedSum = 0;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidAngle.reset();
        pidPos.reset();
        pidSpeed.reset();
        digitalWrite(motEnablePin, 0); // Inverted action on enable pin
        // delay(1);
      }
    }

    motLeft.update();
    motRight.update();
    // updateStepper(&motLeft);
    // updateStepper(&motRight);

    avgBatteryVoltage = avgBatteryVoltage*0.995 + analogRead(battVoltagePin)*0.0293*0.005;

    if (k==plot.prescaler) {
      k = 0;

      if (wsServer.connectedClients(0)>0 && plot.enable) {
        union {
          struct {
            uint8_t cmd = 255;
            uint8_t fill1;
            uint8_t fill2;
            uint8_t fill3;
            uint32_t time;
            float f[11];
          };
          uint8_t b[52];
        } plotData;

        plotData.time = micros();
        plotData.f[0] = accAngle;
        plotData.f[1] = filterAngle;
        plotData.f[2] = pidAngle.setpoint;
        plotData.f[3] = pidAngle.input;
        plotData.f[4] = pidAngleOutput;
        plotData.f[5] = pidPos.setpoint;
        plotData.f[6] = pidPos.input;
        plotData.f[7] = pidPosOutput;
        plotData.f[8] = pidSpeed.setpoint;
        plotData.f[9] = pidSpeed.input;
        plotData.f[10] = pidSpeedOutput;
        wsServer.sendBIN(0, plotData.b, sizeof(plotData.b));
      }
    }
    k++;

    // for (uint8_t i=0; i<6; i++) {
    //   Serial << IBus.readChannel(i) << "\t";
    // }
    // Serial << endl;

    // Serial << microStep << "\t" << absSpeed << "\t" << endl;

    parseSerial();

    tLast = tNow;
  }

  // Run other tasks
  ArduinoOTA.handle();
  IBus.loop();
  wsServer.loop(); // Shouldn't this run on core 0?

}

void parseSerial() {
  static char serialBuf[10];
  static uint8_t pos = 0;
  char currentChar;

  while (Serial.available()) {
    currentChar = Serial.read();
    serialBuf[pos++] = currentChar;
    if (currentChar == 'x') {
      parseCommand(serialBuf, pos);
      pos = 0;
    }
  }

}

void parseCommand(char* data, uint8_t length) {
  float val2;
  if ((data[length-1]=='x') && length>=3) {
    switch (data[0]) {
      case 'c': { // Change controller parameter
        uint8_t controllerNumber = data[1] - '0';
        char cmd2 = data[2];
        float val = atof(data+3);

        // Make a temporary pid object, in which parameters are updated
        PID pidTemp = pidAngle;
        switch (controllerNumber) {
          case 1: pidTemp = pidAngle; break;
          case 2: pidTemp = pidPos;   break;
          case 3: pidTemp = pidSpeed; break;
        }

        switch (cmd2) {
          case 'p': pidTemp.K = val;  break;
          case 'i': pidTemp.Ti = val; break;
          case 'd': pidTemp.Td = val; break;
          case 'n': pidTemp.N = val; break;
          case 't': pidTemp.controllerType = (uint8_t) val; break;
          case 'm': pidTemp.maxOutput = val; break;
          case 'o': pidTemp.minOutput = -val; break;
        }
        pidTemp.updateParameters();

        // Store temporary pid object in correct pid object
        switch (controllerNumber) {
          case 1: pidAngle = pidTemp; break;
          case 2: pidPos = pidTemp;   break;
          case 3: pidSpeed = pidTemp; break;
        }

        Serial << controllerNumber << "\t" << pidTemp.K << "\t" << pidTemp.Ti << "\t" << pidTemp.Td << "\t" << pidTemp.N << "\t" << pidTemp.controllerType << endl;
        break;
      }
      case 'a': // Change angle offset
        angleOffset = atof(data+1);
        Serial << angleOffset << endl;
        break;
      case 'f':
        gyroFilterConstant = atof(data+1);
        Serial << gyroFilterConstant << endl;
        break;
      case 'v':
        motorCurrent = atof(data+1);
        Serial << motorCurrent << endl;
        dacWrite(motorCurrentPin, motorCurrent);
        break;
      case 'm':
        val2 = atof(data+1);
        Serial << val2 << endl;
        controlMode = val2;
        break;
      case 'u':
        microStep = atoi(data+1);
        setMicroStep(microStep);
        break;
      case 'g':
        gyroGain = atof(data+1);
        break;
      case 'h':
        plot.enable = atoi(data+1);
        break;
      case 'i':
        plot.prescaler = atoi(data+1);
        break;
      case 'j':
        gyroGain = atof(data+1);
        break;
      case 'k': {
        uint8_t cmd2 = atoi(data+1);
        if (cmd2==1) {
          calculateGyroOffset(100);
        } else if (cmd2==2) {
          Serial << "Updating angle offset from " << angleOffset;
          angleOffset = filterAngle;
          Serial << " to " << angleOffset << endl;
          EEPROM.writeFloat(EEPROM_ADR_ANGLE_OFFSET, angleOffset);
        }
        break;}
      case 'l':
        maxStepSpeed = atof(&data[1]);
        break;
      case 'n':
        gyroFilterConstant = atof(&data[1]);
        break;
      case 'w': {
        char cmd2 = data[1];
        char buf[30];
        uint8_t len;

        switch (cmd2) {
          case 'r':
            Serial.println("Rebooting...");
            ESP.restart();
            break;
          case 'l': // Send wifi networks to WS client
            sendWifiList();
            break;
          case 's': // Update WiFi SSID
            len = length-3;
            // EEPROM.write(EEPROM_ADR_WIFI_SSID, len);
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            EEPROM.writeString(EEPROM_ADR_WIFI_SSID, buf);
            EEPROM.commit();
            break;
          case 'k': // Update WiFi key
            len = length-3;
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            EEPROM.writeString(EEPROM_ADR_WIFI_KEY, buf);
            EEPROM.commit();
            break;
          case 'm': // WiFi mode (0=AP, 1=use SSID)
            Serial.println(atoi(&data[2]));
            EEPROM.write(EEPROM_ADR_WIFI_MODE, atoi(&data[2]));
            EEPROM.commit();
          }
        break;}
    }
  }
}

void sendWifiList(void) {
  char wBuf[200];
  uint8_t n;
  uint16_t pos = 2;

  wBuf[0] = 'w';
  wBuf[1] = 'l';

  Serial.println("Scan started");
  n = WiFi.scanNetworks();

  if (n>5) n = 5; // Limit to first 5 SSIDs

  // Make concatenated list, separated with commas
  for (uint8_t i=0; i<n; i++) {
    pos += sprintf(wBuf + pos, "%s,", WiFi.SSID(i).c_str());
  }
  wBuf[pos-1] = 0;

  Serial.println(wBuf);
  wsServer.sendTXT(0, wBuf);
}

void calculateGyroOffset(uint8_t nSample) {
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int16_t x, y, z;

  for (uint8_t i=0; i<nSample; i++) {
    imu.getRotation(&x, &y, &z);
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(1);
  }

  gyroOffset[0] = sumX/nSample;
  gyroOffset[1] = sumY/nSample;
  gyroOffset[2] = sumZ/nSample;


  for (uint8_t i=0; i<3; i++) {
    EEPROM.writeShort(EEPROM_ADR_GYRO_OFFSET + i*2, gyroOffset[i]);
  }
  EEPROM.commit();

  Serial << "New gyro calibration values: " << gyroOffset[0] << "\t" << gyroOffset[1] << "\t" << gyroOffset[2] << endl;
}

void readSensor() {
  int16_t ax, ay, az, gx, gy, gz;
  float deltaGyroAngle;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // accAngle = atan2f((float) ax, (float) az) * 180.0/M_PI;
  // deltaGyroAngle = -((float)((gy - gyroOffset[1])) / GYRO_SENSITIVITY) * dT * gyroGain;
    accAngle = atan2f((float) ay, (float) az) * 180.0/M_PI - angleOffset;
    deltaGyroAngle = ((float)((gx - gyroOffset[0])) / GYRO_SENSITIVITY) * dT * gyroGain;

  filterAngle = gyroFilterConstant * (filterAngle + deltaGyroAngle) + (1 - gyroFilterConstant) * (accAngle);

  // Serial << ay/1000.0 << "\t" << az/1000.0 << "\t" << accAngle << "\t" << filterAngle << endl;
}

void initSensor(uint8_t n) {
  float gyroFilterConstantBackup = gyroFilterConstant;
  gyroFilterConstant = 0.8;
  for (uint8_t i=0; i<n; i++) {
    readSensor();
  }
  gyroFilterConstant = gyroFilterConstantBackup;

}

void setMicroStep(uint8_t uStep) {
  // input:                     1 2 4 8 16 32
  // uStep table corresponds to 0 1 2 3 4  5  in binary on uStep pins
  // So, we need to take the log2 of input
  uint8_t uStepPow = 0;
  while (uStep >>= 1) uStepPow++;

  digitalWrite(motUStepPin1, uStepPow&0x01);
  digitalWrite(motUStepPin2, uStepPow&0x02);
  digitalWrite(motUStepPin3, uStepPow&0x04);
}

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
	const uint8_t* src = (const uint8_t*) mem;
	Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) {
		if(i % cols == 0) {
			Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		}
		Serial.printf("%02X ", *src);
		src++;
	}
	Serial.printf("\n");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
                IPAddress ip = wsServer.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);

        				// send message to client
                char wBuf[40];
                sprintf(wBuf, "c%dp%5.2f", 1, pidAngle.K);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%di%5.2f", 1, pidAngle.Ti);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dd%5.2f", 1, pidAngle.Td);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dn%5.2f", 1, pidAngle.N);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dm%5.1f", 1, pidAngle.maxOutput);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%do%5.1f", 1, -pidAngle.minOutput);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dp%5.2f", 2, pidPos.K);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%di%5.2f", 2, pidPos.Ti);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dd%5.2f", 2, pidPos.Td);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dn%5.2f", 2, pidPos.N);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dm%5.1f", 2, pidPos.maxOutput);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%do%5.1f", 2, -pidPos.minOutput);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dp%5.2f", 3, pidSpeed.K);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%di%5.2f", 3, pidSpeed.Ti);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dd%5.2f", 3, pidSpeed.Td);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dn%5.2f", 3, pidSpeed.N);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%dm%5.1f", 3, pidSpeed.maxOutput);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "c%do%5.1f", 3, -pidSpeed.minOutput);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "h%4.2f", speedFilterConstant);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "i%4.2f", steerFilterConstant);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "v%d", motorCurrent);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "j%4.2f", gyroGain);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "n%5.3f", gyroFilterConstant);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "l%5.0f", maxStepSpeed);
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "wm%d", EEPROM.read(EEPROM_ADR_WIFI_MODE));
                wsServer.sendTXT(num, wBuf);
                sprintf(wBuf, "ws%s", EEPROM.readString(EEPROM_ADR_WIFI_SSID).c_str());
                wsServer.sendTXT(num, wBuf);

            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
            parseCommand((char*) payload, length);

            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
            Serial.printf("[%u] get binary length: %u\n", num, length);
            hexdump(payload, length);

            // send message to client
            // webSocket.sendBIN(num, payload, length);
            break;
		case WStype_ERROR:
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
    }

}
