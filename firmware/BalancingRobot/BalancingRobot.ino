#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// MPU6050 객체 선언
MPU6050 mpu;

// MPU6050 데이터를 위한 변수
bool dmpReady = false;//DMP 초기화 완료 여부
uint8_t mpuIntStatus;//MPU 인터럽트 상태
uint8_t devStatus; //MPU 장치 상태
uint16_t packetSize;//센서 데이터 패킷 크기
uint16_t fifoCount;//FIFO 버퍼의 바이트 수
uint8_t fifoBuffer[64];//FIFO 버퍼

// 오일러 각도와 사원수 데이터
Quaternion q;
VectorFloat gravity;
float ypr[3];
float euler[3];

// PID 상수 및 변수 선언
double Kp = 80;
double Ki = 5;
double Kd = 1.5;

double Input, Output, Setpoint;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int BaseSpeed = 0;  // 원하는 이동 속도 (전진 +, 후진 -)

// 모터 핀 설정
const int ENB = 3;
const int IN4 = 4;
const int IN3 = 5;
const int IN2 = 6;
const int IN1 = 7;
const int ENA = 9;

// 로봇의 목표 각도 (대략 0.0)
const float targetAngle = -0.50; 

// 전역 변수로 기울기 각도를 선언
float currentAngle = 0.0;

// MPU6050 인터럽트 핸들러 함수
volatile bool mpuInterrupt = false;
void dmpDataReady() {
 mpuInterrupt = true;
}

void setup() {
//시리얼 통신 시작 (디버깅용)
 Serial.begin(115200);

 myPID.SetSampleTime(10); // 10ms마다 계산 (기본값은 100ms라 너무 느릴 수 있음)

//모터 핀 설정
 pinMode(ENA, OUTPUT);
 pinMode(ENB, OUTPUT);
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);

// MPU6050 초기화
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 Wire.setClock(400000);
#endif
 mpu.initialize();
 devStatus = mpu.dmpInitialize();

 if (devStatus == 0) {
  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
 } else {
// 초기화 실패 시
  Serial.print("DMP Initialization failed (code ");
  Serial.print(devStatus);
  Serial.println(")");
 }
 
// PID 제어기 설정
 Setpoint = targetAngle;
 myPID.SetMode(AUTOMATIC);
 myPID.SetOutputLimits(-255, 255); // PWM 값 범위 (0-255)
}

void loop() {
// MPU6050 데이터가 준비될 때까지 기다림
 if (!dmpReady) return;
 if (!mpuInterrupt && fifoCount < packetSize) return;
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();
 fifoCount = mpu.getFIFOCount();

 if (fifoCount >= packetSize) {
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

// MPU6050의 pitch 각도(기울기)를 입력값으로 사용
  currentAngle = ypr[1] * 180 / M_PI; // 라디안을 각도로 변환
  Input = currentAngle;
 }

// PID 계산
 myPID.Compute();

// 모터 제어
// 최종 PWM 계산
int motorPWM = Output + BaseSpeed;

//if (motorPWM > 0) {
//   motorPWM = map(motorPWM, 0, 255, 70, 255); // 최소 60부터 시작
//} else if (motorPWM < 0) {
//    motorPWM = map(motorPWM, -255, 0, -255, -30); // 최소 -60부터 시작
//}

// PWM 범위 제한
motorPWM = constrain(motorPWM, -255, 255);

if (motorPWM > 0) {
  analogWrite(ENA, motorPWM);
  analogWrite(ENB, motorPWM);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
} else {
  analogWrite(ENA, -motorPWM);
  analogWrite(ENB, -motorPWM);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// 시리얼 플로터용 출력 (이름을 같이 출력하면 플로터에서 구분하기 좋습니다)
Serial.print("Setpoint:");
Serial.print(Setpoint);
Serial.print(",");
Serial.print("Input:");
Serial.print(currentAngle);
Serial.print(",");
Serial.print("Output:");
Serial.println(Output);
}