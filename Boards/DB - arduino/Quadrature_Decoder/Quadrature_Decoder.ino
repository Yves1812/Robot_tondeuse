#include "WProgram.h"
#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
 
// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed creating more than
// 40000 encoder ticks per second per motor.

// Mega external interrupt pins 2, 3, 18, 19, 20, 21
 
// Quadrature encoders
// Front Left encoder
#define FrontLeftEncoderPinA 19
#define FrontLeftEncoderPinB 25
volatile bool LeftEncoderBSet;
volatile long LeftEncoderTicks = 0;
 
// Front Right encoder
#define FrontRightEncoderPinA 18
#define FrontRightEncoderPinB 24
volatile bool RightEncoderBSet;
volatile long RightEncoderTicks = 0;
 
 
void setup()
{
  Serial.begin(115200);
  
  // Quadrature encoders
  // Front Left encoder
  pinMode(FrontLeftEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(frontLeftEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(FrontLeftEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(FrontLeftEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(FrontLeftEncoderPinA), HandleFrontLeftMotorInterruptA, RISING);
 
  // Front Right encoder
  pinMode(FrontRightEncoderPinA, INPUT);      // sets pin A as input
  digitalWrite(FrontRightEncoderPinA, LOW);  // turn on pullup resistors
  pinMode(FrontRightEncoderPinB, INPUT);      // sets pin B as input
  digitalWrite(FrontRightEncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(FrontRightEncoderPinA), HandleFrontRightMotorInterruptA, RISING);
}
 
void loop()
{

}
 
// Interrupt service routines for the front left motor's quadrature encoder
void HandleFrontLeftMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  FrontLeftEncoderBSet = digitalReadFast(FrontLeftEncoderPinB);   // read the input pin
 
  // and adjust counter + if A leads B
  #ifdef LeftEncoderIsReversed
    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
  #else
    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
  #endif
}
 
// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _RightEncoderBSet = digitalReadFast(c_RightEncoderPinB);   // read the input pin
 
  // and adjust counter + if A leads B
  #ifdef RightEncoderIsReversed
    _RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
  #else
    _RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
  #endif
}
