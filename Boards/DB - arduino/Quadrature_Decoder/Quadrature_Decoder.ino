#include <Wire.h>

// Mega external interrupt pins 2, 3, 18, 19, 20, 21
// 2 = FLPulse PORTE4
// 22 = FLDir PORTA0
// 3 = FRPulse PORTE5
// 24 = FRDir PORTA2
// 18 = RLPulse PORTD3
// 52 = RLDir PORTB1
// 19 = RFPulse PORTD2
// 50 = RFDir PORTB3
// 20 = SDA
// 21 = SCL
 
// Quadrature encoders

// Front Left encoder
#define FrontLeftEncoderPulse 2
#define FrontLeftEncoderDir 22 //PORTA 00000001
// Front Right encoder
#define FrontRightEncoderPulse 3
#define FrontRightEncoderDir 24 //PORTA 00000100

// Rear Left encoder
#define RearLeftEncoderPulse 18
#define RearLeftEncoderDir 52 //PORTB 00000010
// Rear Right encoder
#define RearRightEncoderPulse 19
#define RearRightEncoderDir 50 //PORTB 00001000
 
volatile byte ticks[5]={127,127,127,127,0};
volatile byte ticks_latched[5];
volatile unsigned long now, last_time, time_delta;

void setup()
{
  Serial.begin(115200);
  
  // Quadrature encoders
  // Front Left encoder
  pinMode(FrontLeftEncoderPulse, INPUT);      // sets pin A as input
  digitalWrite(FrontLeftEncoderPulse, LOW);  // turn on pullup resistors
  pinMode(FrontLeftEncoderDir, INPUT);      // sets pin B as input
  digitalWrite(FrontLeftEncoderDir, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(FrontLeftEncoderPulse), HandleFrontLeftPulse, RISING);
 
  // Front Right encoder
  pinMode(FrontRightEncoderPulse, INPUT);      // sets pin A as input
  digitalWrite(FrontRightEncoderPulse, LOW);  // turn on pullup resistors
  pinMode(FrontRightEncoderDir, INPUT);      // sets pin B as input
  digitalWrite(FrontRightEncoderDir, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(FrontRightEncoderPulse), HandleFrontRightPulse, RISING);
  
    // Rear Left encoder
  pinMode(RearLeftEncoderPulse, INPUT);      // sets pin A as input
  digitalWrite(RearLeftEncoderPulse, LOW);  // turn on pullup resistors
  pinMode(RearLeftEncoderDir, INPUT);      // sets pin B as input
  digitalWrite(RearLeftEncoderDir, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(RearLeftEncoderPulse), HandleRearLeftPulse, RISING);
 
  // Rear Right encoder
  pinMode(RearRightEncoderPulse, INPUT);      // sets pin A as input
  digitalWrite(RearRightEncoderPulse, LOW);  // turn on pullup resistors
  pinMode(RearRightEncoderDir, INPUT);      // sets pin B as input
  digitalWrite(RearRightEncoderDir, LOW);  // turn on pullup resistors
  attachInterrupt(digitalPinToInterrupt(RearRightEncoderPulse), HandleRearRightPulse, RISING);
  
   // I2C bus setup
   Wire.begin(8);                // join i2c bus with address #8
   Wire.onRequest(requestEvent); // register event
}
 
void loop()
{
   Serial.println(last_time);
}
 
// Interrupt service routines
// I2C write data
void requestEvent() {
   memcpy((byte*)ticks_latched, (byte *) ticks, sizeof(ticks)); // not sure this is needed because I2C interrupt shall not be interruted by external interrupt
   now=millis();
   time_delta=now-last_time;
   last_time=now;
   ticks_latched[4]=(byte) time_delta;
   Wire.write((byte*) ticks_latched, 5); // respond with message of 6 bytes as expected by master
}

// Encoders External interrupts since the interrupt will only fire on 'rising' we don't need to read pulse
// and adjust counter + if A leads B or - if reverse

void HandleFrontLeftPulse()
{
  if ( PORTA & 00000001) {
    ticks[0] += 1;
  }
  else
  {
    ticks[0] -= 1;
  }
}
 
 void HandleFrontRightPulse()
{
  if (PORTA & 00000100) {
    ticks[1] += 1;
  }
  else
  {
    ticks[1] -= 1;
  }
}
 
 void HandleRearLeftPulse()
{
  if (PORTB & 00000010) {
    ticks[2] += 1;
  }
  else
  {
    ticks[2] -= 1;
  }
}
 
 void HandleRearRightPulse()
{
  if ( PORTB & 00001000) {
    ticks[3] += 1;
  }
  else
  {
    ticks[3] -= 1;
  }
}
