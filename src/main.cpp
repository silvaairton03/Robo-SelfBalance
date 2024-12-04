#include<Arduino.h>
#include<Wire.h>
#include<MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include"AccelStepper.h"
#include"MultiStepper.h"
#include "encoder_as5600.h"
#include "AS5600.h"

AccelStepper motor1(1, 25, 26);
#define M1_ENABLE_PIN  19

AccelStepper motor2(1, 32, 33);
#define M2_ENABLE_PIN  18

Encoder encoder;

// TwoWire Wire_1 = TwoWire(1);

AS5600 as5600_0(&Wire);
// AS5600 as5600_1(&Wire_1);

MultiStepper steppers;

MPU6050_Base mpu;

int16_t ax, ay, az, gx, gy, gz;
//void readAccelerometer();
void imuMeasures();
void calculateRollOffset();
// float kalmanAnglePitch = 0, KalmanUncertantyAnglePitch = 2*2;
// float KalmanOutput[] = {0,0}; //A primeira variável é o ângulo medido e a segunda é a incerteza
// void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurements);
float alpha = 0.98;
float y_hp[] = {0, 0}, x_hp[] = {0, 0}, y_lp[] = {0, 0}, x_lp[] = {0, 0};
float const_a = 0.95;
float rpm;
float uL, uR;
int pwmL, pwmR;
float pitch, roll, angularVelocity;
float rollOffset;

void controlSignal(float state1, float state2, float state3, float state4);

unsigned long currentMillis;
unsigned long prevMillis = 0;
const unsigned long sampleTime = 35;

//float k[4] = {-2.5456 ,  -32.1588  , -1.2147 ,  -3.0287};
float k[4] = {-16.9811,  -33.1417,  200.7556,   46.2064};

void setup(){
    pinMode(2, OUTPUT);
    Serial.begin(9600);

    pinMode(M1_ENABLE_PIN, OUTPUT);
    pinMode(M2_ENABLE_PIN, OUTPUT);

    //Inicializando MPU6050
    Wire.begin(21, 22);
    // Wire.setClock(400000);
    as5600_0.begin(5);
    as5600_0.setDirection(AS5600_CLOCK_WISE);
    int a = as5600_0.isConnected();
    Serial.print("Connect: ");
    Serial.println(a);
    // encoder.setEncoder_AS5600(as5600_1, 32,33,25, Wire_1); 

    //Configurando Encoder Motor 1
    // encoder1.setEncoder_AS5600(as5600_0, 21, 22, 5, Wire);

    //Configurando Encoder Motor 2
    // encoder2.setEncoder_AS5600(as5600_1, 32, 33, 25, Wire_1);

    motor1.setMaxSpeed(400);
    motor2.setMaxSpeed(400);

    steppers.addStepper(motor1);
    steppers.addStepper(motor2);
    // mpu.reset();
    // delay(100);
    // mpu.initialize();

    // if (!mpu.testConnection()) {
    //     Serial.println("MPU6050 not connected!");
    //     while (1);
    // }
    // Serial.println("MPU6050 connected!");

    // mpu.setFullScaleAccelRange(2);
    // mpu.setFullScaleGyroRange(250);

    // calculateRollOffset();
    // Serial.print("Roll Offset Calculated: ");
    // Serial.println(rollOffset);
    digitalWrite(M1_ENABLE_PIN, LOW);
    digitalWrite(M2_ENABLE_PIN, LOW);
}

void loop(){
    // long positions[2];

    // positions[0] = 200;
    // positions[1] = 200;
    // steppers.moveTo(positions);
    // steppers.runSpeedToPosition(); 
    // delay(1000);

    // positions[0] = -200;
    // positions[1] = -200;
    // steppers.moveTo(positions);
    // steppers.runSpeedToPosition(); 
    // delay(1000);

    rpm = encoder.getRPM_AS5600(as5600_0);
    Serial.println(rpm);
    delay(500);
}


void calculateRollOffset()
{
    int samples = 200;
    float tempRoll = 0.0;

    for (int i = 0; i < samples; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float ax_g = (float)ax / 4096.0;
        float ay_g = (float)ay / 4096.0;
        float az_g = (float)az / 4096.0;

        tempRoll += atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI;
        delay(10);
    }
    rollOffset = tempRoll / samples;
}

void imuMeasures(){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ax_g = (float)ax / 4096.0;
    float ay_g = (float)ay / 4096.0;
    float az_g = (float)az / 4096.0;

    pitch = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g));
    roll = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) ;

    roll -= rollOffset * PI/180; //Em radianos
    //LOW PASS
    x_lp[0] = roll;
    y_lp[0] = (1 - const_a) * x_lp[1] + const_a * y_lp[1];
    x_lp[1] = x_lp[0];
    y_lp[1] = y_lp[0];
    
    angularVelocity = ((float)gy / 131.0) * (PI / 180.0); //Em radianos
    x_hp[0] = angularVelocity;
    y_hp[0] = const_a * y_hp[1] + x_hp[0] - x_hp[1];
    x_hp[1] = x_hp[0];
    y_hp[1] = y_hp[0];
}
// void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurements)
// {
//   KalmanState = KalmanState + 0.004*KalmanInput; //Prediz o estado atual do sistema
//   KalmanUncertainty = KalmanUncertainty + (0.004 * 0.004 * 4 * 4); //Calcula a incerteza do sistema;
//   float KalmanGain = KalmanUncertainty * 1/ (1 * KalmanUncertainty + 0.044 * 0.044); //Calcula o ganho de Kalman através das incertezas nas predições e medidas
//   KalmanState = KalmanState + KalmanGain * (KalmanMeasurements-KalmanState); //Atualiza o ângulo do sistema com as medidas através do ganho de Kalman
//   KalmanUncertainty = (1 - KalmanGain); //Atualiza a incerteza do estado medido
//   KalmanUncertainty;
//   KalmanOutput[0] = KalmanState;
//   KalmanOutput[1] = KalmanUncertainty;
// }

void controlSignal(float state1, float state2, float state3, float state4)
{
  uL = 0.15 * -(k[0]*state1 + k[1]*state2 + k[2]*state3 + k[3]*state4);
  // uL = 7;
  uR = uL;

  uL = constrain(uL, -9.0, 9.0);
  uR = constrain(uR, -9.0, 9.0);

  pwmL = map(-uL, -9.0, 9.0, -200, 200); 
  pwmR = map(-uR, -9.0, 9.0, -200, 200); 

  // int PWM_MIN = 75;
  // if (uL < 0) {
  //       pwmL = map(uL, -9.0, 0.0, -200, -PWM_MIN); 
  //   } else if (uL > 0) {
  //       pwmL = map(uL, 0.0, 9.0, PWM_MIN, 200); 
  //   } else {
  //       pwmL = 0; 
  //   }

  // if (uR < 0) {
  //       pwmR = map(uR, -9.0, 0.0, -200, -PWM_MIN);
  //   } else if (uR > 0) {
  //       pwmR = map(uR, 0.0, 9.0, PWM_MIN, 200);
  //   } else {
  //       pwmR = 0;
  //   }
}
