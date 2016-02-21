/*

Arduino Code for a Line Following Robot
---------------------------------------
Nicholas Moya and Jon Deboy
April 12th, 2015

This code controls how a robot follows a dark line on a lighter background using PID control.
The sensor reads the position of the robot on the line as a value between 0 and 5000.

0    = furthest left postion
1000 = leaning left
2000 = barely tilting left
2500 = Perfectly centered on line (ideal position)
3000 = barely tilting right
4000 = leaning right
5000 = furthest right position

The error value is calculated as the difference between the current position and the ideal position.
error = current position - ideal postion

PID control is then used to calculate the correction value needed.
P = Kp*error
I = (I + error)*Ki
D = (lastError - error)*Kd
Correction = P + I + D

This correction value is then used to make the motors turn and move appropriately.

rightMotorSpeed = rightBaseSpeed + correction
leftMotorSpeed = leftBaseSpeed - correction
if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed
if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed
if (rightMotorSpeed < 0) rightMotorSpeed = 0
if (leftMotorSpeed < 0) leftMotorSpeed = 0

Lastly, the motorSpeeds are sent as PWM values to power the motors.

rightMotorPWM = rightMotorSpeed
leftMotorPWM = leftMotorSpeed

*/

#include <QTRSensors.h> // This header calls the sensor array library

#define Kp 1 // Propotional Term. To determine this, start wiht a small value that just makes the robot follow the line at a slow speed 
#define Ki 0 // Intergral Term. This value is as zero so technically this robot only uses PD control
#define Kd 20 // Derivative Term. This value should be 20 times larger than the Kp term to compensate for the smaller error difference. To determine this value, slowly increase the speeds and adjust this value.
#define rightMaxSpeed 220 // Max right motor speed of the robot: a safe speed so the robot can travel as fast as possible while still be able to make sharp turns. The highest speed value is 2^8 = 255 (8 bit)
#define leftMaxSpeed 200 // Similarly, this is the max left motot speed of the robot
#define rightBaseSpeed 220 // This is the speed at which the right motor should spin when the robot is perfectly on the line, this acts as a base value to be summed with the correction value
#define leftBaseSpeed 200 // Similarly, this is the same for the left motor
#define NUM_SENSORS 6 // number of sensors used from our sensor array
#define TIMEOUT 2500 // waits for 2500 us for sensor outputs to go low, acts as a grace period between calibration and locomotion
#define EMITTER_PIN 2 // emitter is controlled by digital pin 2, needed for an function argument but the actual pin isn not connected to anything
#define rightMotor1 3 // setting rightMotor1 pin HIGH and rightMotor2 pin LOW will make the motor spin counter clockwise
#define rightMotor2 4 // setting rightMotor2 pin HIGH and rightMotor1 pin LOW will make the motor spin clockwise
#define rightMotorPWM 5 //PWM pin for the right motor
#define leftMotor1 12 // setting leftMotor1 pin HIGH and leftMotor2 pin LOW will make the motor spin counter clockwise
#define leftMotor2 13 // setting leftMotor2 pin HIGH and leftMotor1 pin LOW will make the motor spin clockwise
#define leftMotorPWM 11 //PWM pin for the left motor
#define motorPower 8 // This pin must be HIGH for the motor control IC to work with the Arduino

QTRSensorsRC qtrrc((unsigned char[]) {14, 15, 16, 17, 18, 19} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19
unsigned int sensorValues[NUM_SENSORS]; // sensor values will be gathered from 6/8 sensors

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT); // sets motor pins, PWM pins, and motor power as outputs
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  
  for (int i = 0; i < 100; i++) // calibrates 100 times as the sensor is slid across the track
  {
    qtrrc.calibrate(); // each calibration takes 2.5ms so this process takes 0.25s
  }
  delay(2000); // wait for 2s to position the robot before entering the main loop
} 

int lastError = 0; // initially, there is no previous error

void loop()
{
  unsigned int sensors[NUM_SENSORS]; // sets 6 sensor values to be used
  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2500; // error is the difference between the current position and the ideal position

  int P = error * Kp; // proportional control
  int I = (I + error) * Ki; // integral control
  int D = (error - lastError) * Kd; // derivitive control
  int motorSpeed = P + I + D; // correction set as motor speed needed to compensate for positioning error
  lastError = error; // sets current error as previous error for the next loop

  int rightMotorSpeed = rightBaseSpeed + motorSpeed; // error makes the correction for the robot being too far left (-) and sets the right motor speed to decrease, vice versa is also true
  int leftMotorSpeed = leftBaseSpeed - motorSpeed; // error makes the correction for the robot being too far left (+) and sets the left motor to increase, vice versa is also true
  
  if (rightMotorSpeed > rightMaxSpeed) rightMotorSpeed = rightMaxSpeed; // if the robot is too far right, the right motor goes full speed
  if (leftMotorSpeed > leftMaxSpeed) leftMotorSpeed = leftMaxSpeed; // if the robot is too far left, the left motor goes full speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // if robot is too far left, the right motor stops completely
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // if the robot is too far right, the left motor stops completely
  
  digitalWrite(motorPower, HIGH); // power motor
  digitalWrite(rightMotor1, HIGH); // move right motor forward
  digitalWrite(rightMotor2, LOW); // move right motor forward
  analogWrite(rightMotorPWM, rightMotorSpeed); // move right motor forward with appropriate speeds
  digitalWrite(motorPower, HIGH); // power motor
  digitalWrite(leftMotor1, HIGH); // move left motor forward
  digitalWrite(leftMotor2, LOW); // move left motor forward
  analogWrite(leftMotorPWM, leftMotorSpeed); // move left motor forward with appropriate speeds
}