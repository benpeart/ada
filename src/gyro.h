#ifndef GYRO_H
#define GYRO_H

extern float filterAngle;
extern float angleOffset;

void Gyro_setup(float dT);
void Gyro_CalculateOffset(int nSample);
float Gyro_ReadSensor(float dT);

#endif // GYRO_H