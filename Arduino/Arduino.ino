#include "MeOrion.h"
#include "MeEncoderMotor.h"
#include "MeSerial.h"
#include "MeInfraredReceiver.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#define RELAX_ANGLE 7.5 //自然平衡角度
#define MOTOR_ENABLE     //使能电机

/**************巡线传感器***************/
#define LED_1 !(PINB&(1<<0))  //8  
#define LED_2 !(PIND&(1<<2))  //2
#define LED_3 !(PINB&(1<<4))  //12
#define LED_4 !(PINB&(1<<5))  //13

/***************注册标识位***************/
#define COMDONE 0x0001    //帧指令结束标识
#define MOVING 0x0002     //运动中标识
#define TRACING 0x0004    //巡线标识
#define START 0x0008      //初始化完成标识

MeEncoderMotor encoder;
MeInfraredReceiver ir(PORT_6);

/***************MPU6050变量定义**********************/
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double  compAngleY, gyroYangle;
int16_t tempRaw;
uint32_t IMU_timer;
uint8_t i2cData[14];

/***************PID变量定义**********************/
typedef struct
{
  double P, I, D;
  double Setpoint, Output, Integral, last_error;
  uint32_t Timer;
} PID;

PID  PID_angle, PID_speed, PID_turn;

uint32_t FLAG;
char comdata[19], data_p; //串口缓存数据
float joy_x, joy_y;

void setup()
{
  /*********************初始化通信********************/
  Serial.begin(115299);  //串口
  Wire.begin();           //I2C总线
  ir.begin();
  /*********************MPU6050初始化********************/
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true));
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68)
    while (1);
  delay(100);

  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  tempRaw = (i2cData[6] << 8) | i2cData[7];

  double pitch = atan(-accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  gyroYangle = pitch;
  compAngleY = pitch;
  IMU_timer = micros();

  /*********************编码电机********************/
  encoder.begin();
  encoder.setMode(0, 1);  //pwm直传模式
  encoder.setMode(1, 1);

  /*********************其他初始化********************/
  PID_angle.Setpoint = RELAX_ANGLE;
  PID_angle.P = 22.0;//18
  PID_angle.I = 0.05;//0.1
  PID_angle.D = 0.001;//0.0005

  PID_turn.P = 10;
  PID_turn.D = 30;

  PID_revalue();
  IMU_fillter();
  //FLAG |= TRACING;
  PID_speed.Setpoint = 0;
  gyroYangle = RELAX_ANGLE;
  compAngleY = RELAX_ANGLE;
  FLAG |= START;

}
long receiveTime = 0;
void loop()
{
  if(ir.buttonState()){
    while(ir.available()){
      parseJoystick(ir.read());
      receiveTime = micros();  
    }
  }
  if(micros()-receiveTime>100000){
    receiveTime = micros();
    parseJoystick(0xFA);
  }
  
  IMU_fillter();
  PID_revalue();
  PID_angle_compute();
  PID_speed_compute();
  PID_angle_compute();
}

