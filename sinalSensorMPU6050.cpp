#include "sinalSensorMPU6050.h"
#include <Wire.h>

float gyroX, gyroY, gyroZ;
float gyroCalibrationX, gyroCalibrationY, gyroCalibrationZ;
int calibrationNumber;
float accX, accY, accZ;
float anguloRoll, anguloPitch;
float anguloGyroX, anguloGyroY;

void initMPU6050(){
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void medidasGiroscopio(){
  Wire.beginTransmission(0x68);//Inicia comunicação I2C com MPU6050
  //Filtro Passa-baixas Digital
  Wire.write(0x1A);//Endereço de registrador
  Wire.write(0x05);//Largura de banda de 10Hz
  Wire.endTransmission();
  //Escala de sensibilidade do giroscópio
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);//Endereço do registrador para sensibilidade do giroscópio
  Wire.write(0x8);
  Wire.endTransmission();
  //Acesso aos registradores de medidas do giroscopio
  Wire.beginTransmission(0x68);
  Wire.write(0x43);//Endereço do registrador de medidas
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);//"Puxando informação dos seis registradores disponpiveis na MPU6050"

  int16_t gyX = Wire.read() << 8 | Wire.read();
  int16_t gyY = Wire.read() << 8 | Wire.read();
  int16_t gyZ = Wire.read() << 8 | Wire.read();

  anguloGyroX = gyX/131.0;
  anguloGyroY = gyY/131.0;
}

void calibrationGyro(){
  for (calibrationNumber = 0; calibrationNumber<2000; calibrationNumber ++){
    medidasGiroscopio();
    gyroCalibrationX += gyroX;
    gyroCalibrationY += gyroY;
    gyroCalibrationZ += gyroZ;
  }
  gyroCalibrationX/=2000;
  gyroCalibrationY/=2000;
  gyroCalibrationZ/=2000;
}

void medidasAcelerometro(){
  Wire.beginTransmission(0x68);//Inicia comunicação I2C com MPU6050
  //Filtro Passa-baixas Digital
  Wire.write(0x1A);//Endereço de registrador
  Wire.write(0x05);//Largura de banda de 10Hz
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);//Registrador do Acelerometro
  Wire.write(0x10);
  Wire.endTransmission();
  //Acesso aos registradores de medida do Acelerometro
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read()<<8 | 
    Wire.read();
  int16_t AccYLSB = Wire.read()<<8 | 
    Wire.read();
  int16_t AccZLSB = Wire.read()<<8 | 
    Wire.read();

  accX = (float)AccXLSB/4096;
  accY = (float)AccYLSB/4096;
  accZ = (float)AccZLSB/4096;

  anguloRoll = atan(accY/sqrt(accX*accX + accZ*accZ))*1/(3.142/180); 
  anguloPitch =-atan(accX/sqrt(accY*accY + accZ*accZ))*1/(3.142/180);
}