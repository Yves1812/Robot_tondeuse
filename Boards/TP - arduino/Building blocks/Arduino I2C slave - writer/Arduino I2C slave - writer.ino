// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// This example code is in the public domain - modified by Yves Bonnefont
// v0.1 - 2018-10


#include <Wire.h>

void setup() {
   Wire.begin(8);                // join i2c bus with address #8
   Wire.onRequest(requestEvent); // register event
   Wire.onReceive(receiveEvent); // register event
}

void loop() {
   delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
   Wire.write("hello "); // respond with message of 6 bytes as expected by master
}

void receiveEvent(int howMany) {
   while (1 < Wire.available()) { // loop through all but the last
     char c = Wire.read(); // receive byte as a character
     Serial.print(c);         // print the character
   }
   int x = Wire.read();    // receive byte as an integer
   Serial.println(x);         // print the integer
}
