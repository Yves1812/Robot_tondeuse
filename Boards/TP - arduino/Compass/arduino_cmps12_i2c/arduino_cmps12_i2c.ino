/*****************************************
*     CMPS12 I2C example for Arduino     *
*        By James Henderson, 2014        * 
*****************************************/

#include <Wire.h>

#define CMPS12_ADDRESS 0x60  // Address of CMPS12 shifted right one bit for arduino wire library
#define ANGLE_8  0x1           // Register to read 8bit angle from

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;
int i;

void setup()
{
  Serial.begin(9600);  // Start serial port
  Wire.begin();
  Wire.setClock(100000); 
}

void loop()
{
byte state;
int nbbytes;

int retries=0;
unsigned long now;
 
  // Request 1 byte from the CMPS12
  // this will give us the 8 bit bearing
   while (retries<4) { //checks requested bytes were received 3 retries max
      now=millis();
      Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
      nbbytes=Wire.write(ANGLE_8);//Sends the register we wish to start reading from
      //  Serial.print("nb of bytes written");
      //  Serial.println(nbbytes);
      state=Wire.endTransmission(false);
      //Serial.print("Status of transmission: ");
      //Serial.println(state);
      nbbytes=Wire.requestFrom(CMPS12_ADDRESS, 5);
      //Serial.print("Nb of bytes received: ");
      //Serial.println(nbbytes);
      while (Wire.available()<5 && millis()-now < 100); // need slave to send no less than requested
      //Serial.print("Nb of bytes available");
      //Serial.println(Wire.available());

      if (Wire.available()==5) {
  
        angle8 = Wire.read();               // Read back the 5 bytes
      //  Serial.print("angle8: ");
      //  Serial.println(angle8);
        
        high_byte = Wire.read();
      //  Serial.print("high: ");
      //  Serial.println(high_byte);
      
        low_byte = Wire.read();
      //  Serial.print("low: ");
      //  Serial.println(low_byte);
      
        pitch = Wire.read();
      //  Serial.print("pitch: ");
      //  Serial.println(pitch);
      
        roll = Wire.read();
      //  Serial.print("roll: ");
      //  Serial.println(roll);
        
        angle16 = high_byte;                 // Calculate 16 bit angle
        angle16 <<= 8;
        angle16 += low_byte;
      //  Serial.print("angle16: ");
      //  Serial.println(angle16);
      
        i=i+1;
      //  Serial.print("i: ");
      //  Serial.println(i);
      
        if (i>100)
        {
          i=0;
          Serial.print("roll: ");               // Display roll data
          Serial.print(roll, DEC);
          
          Serial.print("    pitch: ");          // Display pitch data
          Serial.print(pitch, DEC);
          
          Serial.print("    angle full: ");     // Display 16 bit angle with decimal place
          Serial.print(angle16 / 10, DEC);
          Serial.print(".");
          Serial.print(angle16 % 10, DEC);
          
          Serial.print("    angle 8: ");        // Display 8bit angle
          Serial.println(angle8, DEC);
          
          delay(1000);                           // Short delay before next loop
        }
      }
   }
}
