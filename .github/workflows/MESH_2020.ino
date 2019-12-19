#include <QTRSensors.h>

#define Kp 0.3 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 6// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define MaxSpeed 200// max speed of the robot
#define BaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define Stop 0

#define rightMotor1 5
#define rightMotor2 6
#define rightMotorPWM 3
#define leftMotor1 9
#define leftMotor2 8
#define leftMotorPWM 10
#define motorPower 7


QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0,A7,A6,A5,A4,A3,A2,A1}, NUM_SENSORS);
  qtr.setEmitterPin(13);
  
  delay(3000);
  
  int i;
  for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
  {
    move(1, BaseSpeed, 1);//motor derecho hacia adelante
    move(0, BaseSpeed, 0);//motor izquierdo hacia adelante
    qtr.calibrate();   
    delay(20);
  }
  wait();
  delay(3000); // wait for 2s to position the bot before entering the main loop 
}  

int lastError = 0;
unsigned int sensors[8];
int position = qtr.readLineBlack(sensors);

void loop()
{  
  position = qtr.readLineBlack(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position

  if(sensors[0]>900||sensors[7]>900){
    move(1, Stop, 1);//motor derecho hacia adelante
    move(0, Stop, 0);//motor izquierdo hacia adelante
  }
  
  else if(sensors[0]<900 && sensors[7]<900){
    int error = position - 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
  
    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;
    
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;    
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
      
    move(1, rightMotorSpeed, 1);//motor derecho hacia adelante
    move(0, leftMotorSpeed, 1);//motor izquierdo hacia adelante
  }

}
  
void wait(){
  digitalWrite(motorPower, LOW);
}

void move(int motor, int speed, int direction){
  digitalWrite(motorPower, HIGH); //disable standby

  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }  
}
