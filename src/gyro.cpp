#include <Arduino.h>
#include "globals.h"
#include "debug.h"
#include <MPU6050.h>
#include "gyro.h"

// -- IMU
MPU6050 imu;

#define GYRO_SENSITIVITY 65.5

int16_t gyroOffset[3];
float accAngle = 0;
float filterAngle = 0;
float angleOffset = 2.0;
float gyroFilterConstant = 0.996;
float gyroGain = 1.0;

void Gyro_ReadSensor()
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
}

void Gyro_setup()
{
    // Gyro setup (utilize maximum I2C bus speed supported by the MPU6050 - 400kHz)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000UL);
    DB_PRINTLN(imu.testConnection());
    imu.initialize();
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

    // Read gyro offsets
    DB_PRINT("Gyro calibration values: ");
    for (uint8_t i = 0; i < 3; i++)
    {
        char buf[16];
        sprintf(buf, "gyro_offset_%u", i);
        gyroOffset[i] = preferences.getShort(buf, 0);
        DB_PRINTF("%d\t", gyroOffset[i]);
    }
    DB_PRINTLN();

    // Read angle offset
    angleOffset = preferences.getFloat("angle_offset", 0.0);

    // Perform initial gyro measurements
    float gyroFilterConstantBackup = gyroFilterConstant;
    gyroFilterConstant = 0.8;
    for (uint8_t i = 0; i < 50; i++)
    {
        Gyro_ReadSensor();
    }
    gyroFilterConstant = gyroFilterConstantBackup;
}

void Gyro_CalculateOffset(int nSample)
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
