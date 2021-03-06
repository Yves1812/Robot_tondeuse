#include <Wire.h>

// modified to use SPI bus instead of I2C - could be accelerated if needed (optimize memcpy)

// Mega external interrupt pins 2, 3, 18, 19, 20, 21 beware 20 & 21 have pull-ups to 5V
// 2 = FLPulse PORTE4
// 26 = FLDir PORTA4
// 3 = FRPulse PORTE5
// 28 = FRDir PORTA6
// 18 = RLPulse PORTD3
// 30 = RLDir PORTC7
// 19 = RRPulse PORTD2
// 32 = RRDir PORTC5

// 50 = MISO
// 52 = SCK
// 51 = MOSI
// 53 = SS (HW)

// 20 = SDA
// 21 = SCL


// Quadrature encoders pins
// Rear Right encoder
#define RearRightEncoderPulse 2
#define RearRightEncoderDir 32 //PORTC5 
#define RearRightPort PINC
#define RearRightMask B00100000

// Rear Left encoder - marche mais donne RL
#define RearLeftEncoderPulse 3
#define RearLeftEncoderDir 30 //PORTC7
#define RearLeftPort PINC
#define RearLeftMask B10000000

// Front Left encoder
#define FrontLeftEncoderPulse 18
#define FrontLeftEncoderDir 26 //PORTA4
#define FrontLeftPort PINA
#define FrontLeftMask B00010000

// Front Right encoder - marche mais donne FR
#define FrontRightEncoderPulse 19
#define FrontRightEncoderDir 28 //PORTA6
#define FrontRightPort PINA
#define FrontRightMask B01000000
 
// global variables
volatile byte ticks[5]={127,127,127,127,0};
volatile byte ticks_init[5]={127,127,127,127,0};
volatile byte ticks_latched[5]={127,127,127,127,0};
volatile unsigned long now, last_time;
volatile byte time_delta;
unsigned long last_moment=0; // for testing
volatile int myinterrupts[4]={0,0,0,0};

void requestEvent();

// Encoders External interrupts since the interrupt will only fire on 'rising' we don't need to read pulse
// and adjust counter + if A leads B or - if reverse
void HandleFrontLeftPulse()
{  
  if (FrontLeftPort & FrontLeftMask) {
    ticks[0]--;
  }
  else
  {
    ticks[0]++;
  }
}
 
void HandleFrontRightPulse()
{
  if (FrontRightPort & FrontRightMask) {
    ticks[1]++;
  }
  else
  {
    ticks[1]--;
  }
}
 
void HandleRearLeftPulse()
{
  if (RearLeftPort & RearLeftMask) {
    ticks[2]--;
  }
  else
  {
    ticks[2]++;
  }
}
 
void HandleRearRightPulse()
{
  if (RearRightPort & RearRightMask) {
    ticks[3]++;
  }
  else
  {
    ticks[3]--;
  }
}

void setup()
{
  Serial.begin(9600);
  // Set SPI bus
  pinMode(MISO, OUTPUT);     // have to send on master in, *slave out*
  SPCR |= _BV(SPE);   // turn on SPI in slave mode
  SPCR |= _BV(SPIE);   // turn on interrupts
  SPDR=0;
  
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
  
//   // I2C bus setup
//   Wire.begin(8);                // join i2c bus with address #8
//   Wire.onRequest(requestEvent); // register event
}

void loop()
{
//   Serial.println(last_time);
//   if (millis()-last_moment>100){ // for testing purpose
//   last_moment=millis();
//     memcpy((byte*)ticks_latched, (byte *) ticks, sizeof(ticks)); // not sure this is needed because I2C interrupt shall not be interruted by external interrupt
//     memcpy((byte*)ticks, (byte *) ticks_init, sizeof(ticks)); // not sure this is needed because I2C interrupt shall not be interruted by external interrupt
//     Serial.println(last_moment);
//     Serial.println(myinterrupts[0]);
//     Serial.println(myinterrupts[1]);
//     Serial.println(ticks_latched[0]);
//     Serial.println(ticks_latched[1]);
//     Serial.println(ticks_latched[2]);
//     Serial.println(ticks_latched[3]);
//     Serial.println();     
//   }
}
 
// Interrupt service routines
// SPI interrupt routine
ISR (SPI_STC_vect)
{
  byte c = SPDR;
  if (c==255)
  {
  // if order = 255 - copy and latch the ticks, read time, return first byte
    memcpy((byte*)ticks_latched, (byte *) ticks, sizeof(ticks)); // not sure this is needed because I2C interrupt shall not be interruted by external interrupt
    memcpy((byte*)ticks, (byte *) ticks_init, sizeof(ticks)); // not sure this is needed because I2C interrupt shall not be interruted by external interrupt
    SPDR=ticks_latched[0]; // respond first byte
    now=millis();
    time_delta=now-last_time;
    last_time=now;
    ticks_latched[4]=(byte) time_delta;
    return;
  }
  else
  {
    SPDR = ticks_latched[c]; // return corresonding tick
    return;
  } 
}  // end of interrupt service routine (ISR) SPI_STC_vect

//// I2C write data
//void requestEvent() {
   //memcpy((byte*)ticks_latched, (byte *) ticks, sizeof(ticks)); // not sure this is needed because I2C interrupt shall not be interruted by external interrupt
   //memcpy((byte*)ticks, (byte *) ticks_init, sizeof(ticks)); // not sure this is needed because I2C interrupt shall not be interruted by external interrupt
   //now=millis();
   //time_delta=now-last_time;
   //last_time=now;
   //ticks_latched[4]=(byte) time_delta;
   //Wire.write((byte*) ticks_latched, 5); // respond with message of 5 bytes as expected by master
//}


