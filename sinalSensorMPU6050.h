// sinalSensorMPU6050.h
#ifndef SINALSENSORMPU6050_H
#define SINALSENSORMPU6050_H

#include <Wire.h>

extern float gyroX, gyroY, gyroZ;
extern float gyroCalibrationX, gyroCalibrationY, gyroCalibrationZ;
extern int calibrationNumber;

extern float accX, accY, accZ;
extern float anguloRoll, anguloPitch;
extern float anguloGyroX, anguloGyroY;

extern float loopTimer;

void initMPU6050();
void medidasGiroscopio();
void medidasAcelerometro();
void calibrationGyro();

#endif // SINALSENSORMPU6050_H