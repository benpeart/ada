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
#include "globals.h"
#include "debug.h"
#ifdef WEBUI
#include "webui.h"
#endif // WEBUI
#include <MPU6050.h>
#ifdef INPUT_PS3
#include "ps3.h"
#endif // INPUT_PS3
#ifdef INPUT_XBOX
#include "xbox.h"
#endif // INPUT_XBOX
#ifdef BATTERY_VOLTAGE
#include "driver/adc.h"
#include "esp_adc_cal.h"
#endif // BATTERY_VOLTAGE

// Driving behaviour
float speedFactor = 0.7;         // how strong it reacts to inputs, lower = softer (limits max speed) (between 0 and 1)
float steerFactor = 1.0;         // how strong it reacts to inputs, lower = softer (limits max speed) (between 0 and 1)
float speedFilterConstant = 0.9; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)
float steerFilterConstant = 0.9; // how fast it reacts to inputs, higher = softer (between 0 and 1, but not 0 or 1)

// #define STEPPER_DRIVER_A4988 // Use A4988 stepper driver, which uses different microstepping settings
#define STEPPER_DRIVER_TMC2209 // Use TMC2209 stepper driver, which uses different microstepping settings

// Remote control structure
remoteControlType remoteControl;

// ----- Function prototypes
void parseSerial();
void calculateGyroOffset(uint8_t nSample);
void readSensor();
void initSensor(uint8_t n);
void setMicroStep(uint8_t uStep);
void sendConfigurationData(uint8_t num);

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();

// -- EEPROM
Preferences preferences;
#define PREF_VERSION 1 // if setting structure has been changed, count this number up to delete all settings
#define PREF_NAMESPACE "pref"
#define PREF_KEY_VERSION "ver"

// -- Stepper motor pin assignments
#define motEnablePin 19

#define motLeftUStepPin1 18
#define motLeftUStepPin2 05
#define motLeftStepPin 33
#define motLeftDirPin 32

#define motRightUStepPin1 04
#define motRightUStepPin2 27
#define motRightStepPin 26
#define motRightDirPin 25

fastStepper motLeft(motLeftStepPin, motLeftDirPin, 0, motLeftTimerFunction);
fastStepper motRight(motRightStepPin, motRightDirPin, 1, motRightTimerFunction);

uint8_t microStep = 16;
float maxStepSpeed = 1500;

// -- PID control
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS / 1000000.0

#define PID_ANGLE 0
#define PID_POS 1
#define PID_SPEED 2

#define PID_ANGLE_MAX 12
PID pidAngle(cPID, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
#define PID_POS_MAX 35
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
PID pidSpeed(cP, dT, PID_POS_MAX, -PID_POS_MAX);

// make these global so we can use them in webui.cpp
float pidAngleOutput;
float pidPosOutput;
float pidSpeedOutput;

enum controlType
{
  ANGLE_ONLY = 0,
  ANGLE_PLUS_POSITION,
  ANGLE_PLUS_SPEED
};
controlType controlMode = ANGLE_PLUS_POSITION;

// Threshold for fall detection. If integral of error of angle controller is larger than this value, controller is disabled
#define angleErrorIntegralThreshold 30.0
#define angleErrorIntegralThresholdDuringSelfright angleErrorIntegralThreshold * 3
#define angleEnableThreshold 5.0   // If (absolute) robot angle is below this threshold, enable control
#define angleDisableThreshold 70.0 // If (absolute) robot angle is above this threshold, disable control (robot has fallen down)

// -- IMU
MPU6050 imu;

#define GYRO_SENSITIVITY 65.5

int16_t gyroOffset[3];
float accAngle = 0;
float filterAngle = 0;
float angleOffset = 2.0;
float gyroFilterConstant = 0.996;
float gyroGain = 1.0;

// Temporary values for debugging sensor algorithm
float rxg, ayg, azg;

// -- Others
#ifdef LED_PINS
#define PIN_LED 32
#define PIN_LED_LEFT 33
#define PIN_LED_RIGHT 26
#endif // LED_PINS

// ADC definitions (for reading battery voltage)
#ifdef BATTERY_VOLTAGE
#define ADC_CHANNEL_BATTERY_VOLTAGE ADC1_CHANNEL_6 // GPIO number 34
// Battery voltage is measured via a 100 and 3.3 kOhm resistor divider. Reference voltage is 1.1 V (if attenuation is set to 0dB)
#define BATTERY_VOLTAGE_SCALING_FACTOR (100 + 3.3) / 3.3
#define BATTERY_VOLTAGE_FILTER_COEFFICIENT 0.99
esp_adc_cal_characteristics_t adc_chars;
#endif // BATTERY_VOLTAGE

// BT MAC
char BTaddress[20] = "00:00:00:00:00:00";

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

// optimize unnecessary calls to digitalWrite as it is relatively slow
void motEnable(boolean enable)
{
  static boolean enabled = true;

  if (enabled != enable)
  {
    digitalWrite(motEnablePin, !enable); // Inverted action on enable pin
    enabled = enable;
  }
}

// ----- Main code
void setup()
{
  Serial.begin(115200);

	// Init EEPROM, if not done before
	preferences.begin(PREF_NAMESPACE, false); // false = RW-mode
	if (preferences.getUInt(PREF_KEY_VERSION, 0) != PREF_VERSION)
	{
		preferences.clear(); // Remove all preferences under the opened namespace
		preferences.putUInt(PREF_KEY_VERSION, PREF_VERSION);
		DB_PRINTF("EEPROM init complete, all preferences deleted, new pref_version: %d\n", PREF_VERSION);
	}

  pinMode(motEnablePin, OUTPUT);
  pinMode(motLeftUStepPin1, OUTPUT);
  pinMode(motLeftUStepPin2, OUTPUT);
  pinMode(motRightUStepPin1, OUTPUT);
  pinMode(motRightUStepPin2, OUTPUT);
  motEnable(false); // Disable steppers during startup
  setMicroStep(microStep);

#ifdef LED_PINS
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED_LEFT, OUTPUT);
  pinMode(PIN_LED_RIGHT, OUTPUT);
  digitalWrite(PIN_LED, 0);
  digitalWrite(PIN_LED_LEFT, 1); // Turn on one LED to indicate we are live
  digitalWrite(PIN_LED_RIGHT, 0);
#endif // LED_PINS
  motLeft.init();
  motRight.init();
  motLeft.microStep = microStep;
  motRight.microStep = microStep;

  // Gyro setup
  //  delay(200);
  Wire.begin(21, 22, 400000UL);
  //  delay(100);
  DB_PRINTLN(imu.testConnection());
  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  // Calculate and store gyro offsets
  //  delay(50);

  // Read gyro offsets
  DB_PRINT("Gyro calibration values: ");
  for (uint8_t i = 0; i < 3; i++)
  {
    char buf[16];
    sprintf(buf, "gyro_offset_%u", i);
    gyroOffset[i] = preferences.getShort(buf, 0);
    DB_PRINTF("%u\t", gyroOffset[i]);
  }
  DB_PRINTLN();

  // Read angle offset
  angleOffset = preferences.getFloat("angle_offset", 0.0);

  // Perform initial gyro measurements
  initSensor(50);

#ifdef WEBUI
  WebUI_setup();
#endif // WEBUI

  pidAngle.setParameters(0.65, 1.0, 0.075, 15);
  pidPos.setParameters(1, 0, 1.2, 50);
  pidSpeed.setParameters(6, 5, 0, 20);

// Setup PS3 controller
#ifdef INPUT_PS3
  ps3_setup();
#endif

// Setup Xbox controller
#ifdef INPUT_XBOX
#endif

  DB_PRINTLN("Ready");

  // Characterize ADC at particular atten
#ifdef BATTERY_VOLTAGE
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_0db, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
  {
    DB_PRINTLN("eFuse Vref");
  }
  else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
  {
    DB_PRINTLN("Two Point");
  }
  else
  {
    DB_PRINTLN("Default");
  }
  Serial << "ADC calibration values (attenuation, vref, coeff a, coeff b):" << adc_chars.atten << "\t" << adc_chars.vref << "\t" << adc_chars.coeff_a << "\t" << adc_chars.coeff_b << endl;

  // Configure ADC
  adc1_config_channel_atten(ADC_CHANNEL_BATTERY_VOLTAGE, ADC_ATTEN_0db);
  adc_set_data_inv(ADC_UNIT_1, true); // For some reason, data is inverted...
#endif                                // BATTERY_VOLTAGE
  DB_PRINTLN("Booted, ready for driving!");
#ifdef LED_PINS
  digitalWrite(PIN_LED_RIGHT, 1);
#endif // LED_PINS
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{
  static unsigned long tLast = 0;
  float avgMotSpeed;
  float steer = 0;
  static float avgSteer;
  static float avgSpeed;
  static boolean enableControl = 0;
  static float avgMotSpeedSum = 0;
  int32_t avgMotStep;
  static float avgBatteryVoltage = 0;
  static uint32_t lastInputTime = 0;
  uint32_t tNowMs;
  float absSpeed = 0;
  float noiseValue = 0;
  static boolean overrideMode = 0, lastOverrideMode = 0;
  static boolean selfRight = 0;
  static boolean disableControl = 0;
  static float angleErrorIntegral = 0;

  unsigned long tNow = micros();
  tNowMs = millis();

  if (tNow - tLast > dT_MICROSECONDS)
  {
    readSensor();

    if (remoteControl.selfRight && !enableControl)
    { // Start self-right action (stops when robot is upright)
      selfRight = 1;
      disableControl = 0;
      remoteControl.selfRight = 0; // Reset single action bool
    }
    else if (remoteControl.disableControl && enableControl)
    { // Sort of kill-switch
      disableControl = 1;
      selfRight = 0;
      remoteControl.disableControl = 0;
    }

    // Filter speed and steer input
    avgSpeed = speedFilterConstant * avgSpeed + (1 - speedFilterConstant) * remoteControl.speed / 5.0;
    avgSteer = steerFilterConstant * avgSteer + (1 - steerFilterConstant) * remoteControl.steer;

    if (enableControl)
    {
      if (abs(avgSpeed) < 0.2)
      {
        // remoteControl.speed = 0;
      }
      else
      {
        lastInputTime = tNowMs;
        if (controlMode == ANGLE_PLUS_POSITION)
        {
          controlMode = ANGLE_PLUS_SPEED;
          motLeft.setStep(0);
          motRight.setStep(0);
          pidSpeed.reset();
        }
      }

      steer = avgSteer;
      // if (abs(avgSteer)>1) {
      //   steer = avgSteer * (1 - abs(avgSpeed)/150.0);
      // } else {
      //   steer = 0;
      // }

      // }

      // Switch to position control if no input is received for a certain amount of time
      if (tNowMs - lastInputTime > 2000 && controlMode == ANGLE_PLUS_SPEED)
      {
        controlMode = ANGLE_PLUS_POSITION;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.reset();
      }

      // Actual controller computations
      if (controlMode == ANGLE_ONLY)
      {
        pidAngle.setpoint = avgSpeed * 2;
      }
      else if (controlMode == ANGLE_PLUS_POSITION)
      {
        avgMotStep = (motLeft.getStep() + motRight.getStep()) / 2;
        pidPos.setpoint = avgSpeed;
        pidPos.input = -((float)avgMotStep) / 1000.0;
        pidPosOutput = pidPos.calculate();
        pidAngle.setpoint = pidPosOutput;
      }
      else if (controlMode == ANGLE_PLUS_SPEED)
      {
        pidSpeed.setpoint = avgSpeed;
        pidSpeed.input = -avgMotSpeedSum / 100.0;
        pidSpeedOutput = pidSpeed.calculate();
        pidAngle.setpoint = pidSpeedOutput;
      }

      pidAngle.input = filterAngle;

      pidAngleOutput = pidAngle.calculate();

      avgMotSpeedSum += pidAngleOutput / 2;
      if (avgMotSpeedSum > maxStepSpeed)
      {
        avgMotSpeedSum = maxStepSpeed;
      }
      else if (avgMotSpeedSum < -maxStepSpeed)
      {
        avgMotSpeedSum = -maxStepSpeed;
      }
      avgMotSpeed = avgMotSpeedSum;
      motLeft.speed = avgMotSpeed + steer;
      motRight.speed = avgMotSpeed - steer;

      // Detect if robot has fallen. Concept: integrate angle controller error over time.
      // If absolute integrated error surpasses threshold, disable controller
      angleErrorIntegral += (pidAngle.setpoint - pidAngle.input) * dT;
      if (selfRight)
      {
        if (abs(angleErrorIntegral) > angleErrorIntegralThresholdDuringSelfright)
        {
          selfRight = 0;
          disableControl = 1;
        }
      }
      else
      {
        if (abs(angleErrorIntegral) > angleErrorIntegralThreshold)
        {
          disableControl = 1;
        }
      }

      // Switch microstepping
      absSpeed = abs(avgMotSpeed);
      uint8_t lastMicroStep = microStep;

      if (absSpeed > (150 * 32 / microStep) && microStep > 1)
        microStep /= 2;
      if (absSpeed < (130 * 32 / microStep) && microStep < 32)
        microStep *= 2;

      // if (microStep!=lastMicroStep) {
      //   motLeft.microStep = microStep;
      //   motRight.microStep = microStep;
      //   setMicroStep(microStep);
      // }

      DB_PRINTF(">motLeft.speed:%d\n", (int)motLeft.speed);
      DB_PRINTF(">motRight.speed:%d\n", (int)motRight.speed);
      DB_PRINTF(">microStep:%d\n", microStep);
      DB_PRINTF(">filterAngle:%d\n", (int)filterAngle);

      // Disable control if robot is almost horizontal. Re-enable if upright.
      if ((abs(filterAngle) > angleDisableThreshold && !selfRight) || disableControl)
      {
        enableControl = 0;
        // disableControl = 0; // Reset disableControl flag
        motLeft.speed = 0;
        motRight.speed = 0;
        motEnable(true);
#ifdef LED_PINS
        digitalWrite(PIN_LED_LEFT, 0);
        digitalWrite(PIN_LED_RIGHT, 0);
#endif // LED_PINS
      }
      if (abs(filterAngle) < angleEnableThreshold && selfRight)
      {
        selfRight = 0;
        angleErrorIntegral = 0; // Reset, otherwise the fall detection will be triggered immediately
      }
    }
    else
    { // Control not active
      DB_PRINTLN("control not active");

      // Override control
      if (overrideMode && !lastOverrideMode)
      { // Transition from disable to enable
        // Enable override mode
        motLeft.speed = 0;
        motRight.speed = 0;
        motEnable(true); // Enable motors
        overrideMode = 1;
      }
      else if (!overrideMode && lastOverrideMode)
      {
        motEnable(false); // disable motors
        overrideMode = 0;
      }
      lastOverrideMode = overrideMode;

      if (abs(filterAngle) > angleEnableThreshold + 5)
      { // Only reset disableControl flag if angle is out of "enable" zone, otherwise robot will keep cycling between enable and disable states
        disableControl = 0;
      }

      if ((abs(filterAngle) < angleEnableThreshold || selfRight) && !disableControl)
      { // (re-)enable and reset stuff
        enableControl = 1;
#ifdef LED_PINS
        digitalWrite(PIN_LED_LEFT, 1);
        digitalWrite(PIN_LED_RIGHT, 1);
#endif // LED_PINS
        controlMode = ANGLE_PLUS_POSITION;
        // avgMotSpeedSum = 0;

        if (!overrideMode)
        {
          avgMotSpeedSum = 0;
          motEnable(true); // Enable motors
          pidAngle.reset();
        }
        else
        {
          avgMotSpeedSum = (motLeft.speed + motRight.speed) / 2;
          overrideMode = 0;
        }

        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.reset();
        pidSpeed.reset();

        angleErrorIntegral = 0;
        // delay(1);
      }

      if (overrideMode)
      {
        float spd = avgSpeed;
        float str = avgSteer;
        // if (spd<3) spd = 0;
        // if (str<3) str = 0;
        motLeft.speed = -30 * spd + 2 * str;
        motRight.speed = -30 * spd - 2 * str;

        // Run angle PID controller in background, such that it matches when controller takes over, if needed
        pidAngle.input = filterAngle;
        pidAngleOutput = pidAngle.calculate();
        // pidSpeed.setpoint = avgSpeed;
        // pidSpeed.input = -(motLeft.speed+motRight.speed)/2/100.0;
        // pidSpeedOutput = pidSpeed.calculate();
      }
      // Serial << motLeft.speed << "\t" << motRight.speed << "\t" << overrideMode << endl;
    }

    motLeft.update();
    motRight.update();
    // updateStepper(&motLeft);
    // updateStepper(&motRight);

#ifdef WEBUI
    WebUI_loop();
#endif // WEBUI    

    parseSerial();

    // Serial << micros()-tNow << "\t";

    tLast = tNow;

    // Handle PS3 controller
#ifdef INPUT_PS3
    Ps3_loop();
#endif

    // Handle Xbox controller
#ifdef INPUT_XBOX
    Xbox_loop();
#endif

    // Serial << micros()-tNow << endl;
  }

  // delay(1);
}

void parseSerial()
{
  static char serialBuf[63];
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
}

void parseCommand(char *data, uint8_t length)
{
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
    case 'm':
      val2 = atof(data + 1);
      DB_PRINTLN(val2);
      controlMode = (controlType)val2;
      break;
    case 'u':
      microStep = atoi(data + 1);
      setMicroStep(microStep);
      break;
    case 'g':
      gyroGain = atof(data + 1);
      break;
    case 'p':
    {
      switch (data[1])
      {
#ifdef WEBUI
      case 'e':
        plot.enable = atoi(data + 2);
        break;
      case 'p':
        plot.prescaler = atoi(data + 2);
        break;
#endif // WEBUI
      }
      break;
    }
    // case 'h':
    //   plot.enable = atoi(data+1);
    //   break;
    // case 'i':
    //   plot.prescaler = atoi(data+1);
    //   break;
    case 'j':
      gyroGain = atof(data + 1);
      break;
    case 'k':
    {
      uint8_t cmd2 = atoi(data + 1);
      if (cmd2 == 1)
      { // calibrate gyro
        calculateGyroOffset(100);
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
#ifdef WEBUI
      case 'l': // Send wifi networks to WS client
        sendWifiList();
        break;
      case 's': // Update WiFi SSID
        len = length - 3;
        memcpy(buf, &data[2], len);
        buf[len] = 0;
        preferences.putBytes("wifi_ssid", buf, 63);
        DB_PRINTF("Updated WiFi SSID to: %s\n", buf);
        break;
      case 'k': // Update WiFi key
        len = length - 3;
        memcpy(buf, &data[2], len);
        buf[len] = 0;
        preferences.putBytes("wifi_key", buf, 63);
        DB_PRINTF("Updated WiFi key to: %s\n", buf);
        break;
      case 'm': // WiFi mode (0=AP, 1=use SSID)
        preferences.putUInt("wifi_mode", atoi(&data[2]));
        DB_PRINTF("Updated WiFi mode to (0=access point, 1=connect to SSID): %d\n", atoi(&data[2]));
        break;
#endif          // WEBUI
      case 'n': // Robot name
        len = length - 3;
        memcpy(buf, &data[2], len);
        buf[len] = 0;
        if (len >= 8)
        {
          preferences.putBytes("robot_name", buf, 63);
        }
        DB_PRINTF("Updated robot name to: %s\n", buf);
        break;
      }
      break;
    }
    }
  }
}

void calculateGyroOffset(uint8_t nSample)
{
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int16_t x, y, z;

  for (uint8_t i = 0; i < nSample; i++)
  {
    imu.getRotation(&x, &y, &z);
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(5);
  }

  gyroOffset[0] = sumX / nSample;
  gyroOffset[1] = sumY / nSample;
  gyroOffset[2] = sumZ / nSample;

  for (uint8_t i = 0; i < 3; i++)
  {
    char buf[16];
    sprintf(buf, "gyro_offset_%u", i);
    preferences.putShort(buf, gyroOffset[i]);
  }

  DB_PRINTF("New gyro calibration values: %d\t%d\t%d\n", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
}

void readSensor()
{
  int16_t ax, ay, az, gx, gy, gz;
  float deltaGyroAngle;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // use this pair if the MPU6050 is mounted in a breadboard across the width of the robot (row of pins to the back)
  accAngle = atan2f((float)ax, (float)az) * 180.0 / M_PI - angleOffset;
  deltaGyroAngle = -((float)((gy - gyroOffset[1])) / GYRO_SENSITIVITY) * dT * gyroGain;

  // use this pair if the MPU6050 is mounted perpendicular to the width of the robot (row of pins underneath the USB port on the ESP32)
  // accAngle = atan2f((float)ay, (float)az) * 180.0 / M_PI - angleOffset;
  // deltaGyroAngle = ((float)((gx - gyroOffset[0])) / GYRO_SENSITIVITY) * dT * gyroGain;

  filterAngle = gyroFilterConstant * (filterAngle + deltaGyroAngle) + (1 - gyroFilterConstant) * (accAngle);

  // Serial << ay/1000.0 << "\t" << az/1000.0 << "\t" << accAngle << "\t" << filterAngle << endl;
  ayg = (ay * 9.81) / 16384.0;
  azg = (az * 9.81) / 16384.0;
  rxg = ((float)((gx - gyroOffset[0])) / GYRO_SENSITIVITY);
  // Serial << ayf << "\t"<< azf << "\t" << accAngle << endl;
}

void initSensor(uint8_t n)
{
  float gyroFilterConstantBackup = gyroFilterConstant;
  gyroFilterConstant = 0.8;
  for (uint8_t i = 0; i < n; i++)
  {
    readSensor();
  }
  gyroFilterConstant = gyroFilterConstantBackup;
}

void setMicroStep(uint8_t uStep)
{
#ifdef STEPPER_DRIVER_TMC2209
  uint8_t ms1, ms2;
  switch (uStep)
  {
  case 8:
    ms1 = 0;
    ms2 = 0;
    break;
  case 16:
    ms1 = 1;
    ms2 = 1;
    break;
  case 32:
    ms1 = 1;
    ms2 = 0;
    break;
  case 64:
    ms1 = 0;
    ms2 = 1;
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
    digitalWrite(motLeftUStepPin1, 1);
    digitalWrite(motLeftUStepPin2, 1);
    digitalWrite(motRightUStepPin1, 1);
    digitalWrite(motRightUStepPin2, 1);
  }
#endif
#endif
}
