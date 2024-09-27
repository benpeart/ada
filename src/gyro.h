#ifndef GYRO_H
#define GYRO_H

extern float dT;
extern float angleOffset;

void Gyro_setup();
void Gyro_CalculateOffset(int nSample);
void Gyro_ReadSensor();

#endif // GYRO_H