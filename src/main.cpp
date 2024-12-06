#include<Arduino.h>
#include<Wire.h>
#include<MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include<AccelStepper.h>
#include<MultiStepper.h>
#include<AS5600.h>

const int EN = 23;
const int DIR = 14;
const int STEP = 12;

const int DIR_2 = 33;
const int STEP_2 = 32;

void acionaMotor();

TwoWire I2C_2 = TwoWire(1); // Criação do segundo barramento I2C

AS5600 as5600(&Wire);;        // Sensor no barramento I2C principal (Wire)
AS5600 as5600_2(&I2C_2);      // Sensor no barramento I2C secundário

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
float pitch, roll, angularVelocity;
float rollOffset;

void controlSignal(float state1, float state2, float state3, float state4);

unsigned long currentMillis;
unsigned long prevMillis = 0;
unsigned long timer = 0;
const unsigned long sampleTime = 35;

//float k[4] = {-2.5456 ,  -32.1588  , -1.2147 ,  -3.0287};
float k[4] = {-16.9811,  -33.1417,  200.7556,   46.2064};

void setup(){
    pinMode(2, OUTPUT);
    Serial.begin(9600);
    pinMode(EN, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(STEP, OUTPUT);

    pinMode(EN, OUTPUT);
    pinMode(DIR_2, OUTPUT);
    pinMode(STEP_2, OUTPUT);

    Serial.println(__FILE__);
    Serial.print("AS5600_LIB_VERSION: ");
    Serial.println(AS5600_LIB_VERSION);
    
    // Configuração do primeiro barramento I2C
    // SCL, SDA
    Wire.begin(21, 22);
    as5600.begin(5); // Configuração do sensor no barramento principal
    as5600.setDirection(AS5600_CLOCK_WISE);

    //Teste de conexão para encoder ligado no primeiro barramento I2C
    Serial.print("Sensor 1 conectado: ");
    Serial.println(as5600.isConnected());

    // Configuração do segundo barramento I2C
    //  SDA, SCL
    I2C_2.begin(26, 25); // Pinos diferentes para o segundo barramento I2C
    as5600_2.begin(4); // Configuração do sensor no segundo barramento
    as5600_2.setDirection(AS5600_CLOCK_WISE);

    Serial.print("Sensor 2 conectado: ");
    Serial.println(as5600_2.isConnected());

    //Inicializando MPU6050
    Wire.setClock(400000);
    mpu.reset();
    delay(100);
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 not connected!");
        while (1);
    }
    Serial.println("MPU6050 connected!");

    mpu.setFullScaleAccelRange(2);
    mpu.setFullScaleGyroRange(250);

    calculateRollOffset();
    Serial.print("Roll Offset Calculated: ");
    Serial.println(rollOffset);

    digitalWrite(EN, LOW);
    for(int i=0;i<2;i++){
        delay(1000);
        acionaMotor();
        delay(1000);
    }
    delay(1000);
}

void loop(){
    //m1.runSpeed();
  if ((millis() - timer) >= 20) {
    acionaMotor();
    // Leituras do primeiro sensor
    Serial.print("Sensor 1 - a = ");
//    Serial.print(as5600.readAngle());
//    Serial.print("\tω = ");
    Serial.println(as5600.getAngularSpeed(AS5600_MODE_RPM));
    // Leituras do segundo sensor
    Serial.print("    Sensor 2 - a = ");
//    Serial.println(as5600_2.readAngle());
//    Serial.print("\tω = ");
    Serial.println(as5600_2.getAngularSpeed(AS5600_MODE_RPM));

    timer = millis();
  }
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

void acionaMotor()
{
    // permite que o motor se mova em uma direção particular
  digitalWrite(DIR, HIGH);
  digitalWrite(DIR_2, HIGH);

  // faz 200 pulsos para fazer uma rotação de ciclo completo
  for (int x = 0; x < 100; x++) {
    digitalWrite(STEP, HIGH);
    digitalWrite(STEP_2, HIGH);
    delayMicroseconds(1500);
    digitalWrite(STEP, LOW);
    digitalWrite(STEP_2, LOW);
    delayMicroseconds(1500);
  }
//  delay(1000); //1 segundo de delay para inverter a direção

//  digitalWrite(DIR, LOW);

//  // faz 400 pulsos para fazer duas rotações de ciclo completo
//  for (int x = 0; x < 200; x++) {
//    digitalWrite(STEP, HIGH);
//    delayMicroseconds(1500);
//    digitalWrite(STEP, LOW);
//    delayMicroseconds(1500);
//  }
//  delay(1000);

}

void imuMeasures()
{
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
{}
