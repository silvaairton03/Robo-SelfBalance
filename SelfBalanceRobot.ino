
#include <Wire.h>
#include "sinalSensorMPU6050.h"
#include "Kalman.h"

Kalman KalmanX;
Kalman KalmanY;
Kalman KalmanZ;

float anguloKalmanX, anguloKalmanY;

uint32_t timer;

void setup(){

  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  initMPU6050();
  delay(100);

  medidasAcelerometro();
  KalmanX.setAngle(anguloRoll);
  KalmanY.setAngle(anguloPitch);

  anguloGyroX = anguloRoll;
  anguloGyroY = anguloPitch;

  timer = micros();
}

void loop(){
  medidasGiroscopio();

  float dt = (float)(micros() - timer)/1000000;
  timer = micros();

  medidasAcelerometro();

  anguloKalmanX = KalmanX.getAngle(anguloRoll, anguloGyroX, dt);
  anguloKalmanY = KalmanY.getAngle(anguloPitch, anguloGyroY, dt);

  Serial.print(anguloRoll);Serial.print(",");
  Serial.print(anguloPitch);
  Serial.print("\n");
  delay(50);
}



