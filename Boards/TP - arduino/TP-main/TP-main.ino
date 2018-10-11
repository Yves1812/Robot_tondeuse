// 2018-10 Yves Bonnefont

#include <Wire.h>

#define DB_address 0x8
#define Compass_address 0x60

struct roverType {
	long x;
	long y;
	int vx;
	int vy;
	byte bearing;
	int teta_point;
}

struct segmentOrder{
  int segment_type;
  int target_ticks;
  int target_bearing;
  int taget_speed;
}

struct segmentStatus {
	int FL_ticks_cum;
	int FR_ticks_cum;
	int RL_ticks_cum;
	int RR_ticks_cum;
  int gap_cum;
	int millis_cum;

  int FL_ticks_step;
	int FR_ticks_step;
	int RL_ticks_step;
	int RR_ticks_step;
  int gap_step;
	int millis_step;
	
	int last_bearing;
  int current_bearing;
}

int I2C_buffer[5];
roverType rover;
segmentStatus segment;
segmentOrder target_move; next_move;

void setup() {
   Wire.begin();        // join i2c bus (address optional for master)
   Serial.begin(9600);  // start serial for output
}

bool readDecoders() {
   byte retries=0;
   unsigned long now;
   
   while (retries<4) { //checks 5 bytes were received 3 retries max
      now=millis();
      Wire.requestFrom(DB_address, 5);    // request 5 bytes from slave device DB_board
      while (Wire.available()<5 && millis()-now < 100); // need slave to send no less than requested
      if (Wire.available()== 5) {
         segment.FL_ticksStep=((int) Wire.read())-127;
         segment.FR_ticksStep=((int) Wire.read())-127;
         segment.RL_ticksStep=((int) Wire.read())-127;
         segment.RR_ticksStep=((int) Wire.read())-127;
         segment.millisStep=buffer_I2C[4];
         return true;
      }
      retries++;
   }
   return false;
}

void deliverStraightSegment() {

  segment.cum_gap=min(segment.FL_ticks_step, segment.RL_ticks_step)-min(segment.FR_ticks_step, segment.RR_ticks_step);
  if (segment.cum_gap>=2){
    L_PWM=L_PWM-1; // Slow left side
  }
  if (segment.cum_gap<=-2){
    R_PWM=R_PWM-1; // Slow right side
  }
  if {
    if (current_gap+previous_gap==0): {
      current_gap=0;
    }
  }
  

  o  If max (current gap; abs(sumof(current gap and previous gap))) between slowest left and slowest right >= 2 ticks (6% deviation):
 slowdown fastest side (function of the gap tbd)
 Reset current gap #fixed so do not carry forward
o Else if sumof(current gap and previous gap)=0 :
 Reset current gap #current and previous cancel out => do not carry forward
o If gap between front and rear
 Slowdown slipping wheel to slowest wheel of the corresponding/both side(s)
o If no gap front rear and abs(sum(current gap – previous gap) <=1:
 Increase speed by 10% on all
o Former_gap=current_gap

	segment.FL_ticksCum+=segment.FL_ticksStep;
	segment.FR_ticksCum+=segment.FR_ticksStep;
	segment.RL_ticksCum+=segment.RL_ticksStep;
	segment.RR_ticksCum+=segment.RR_ticksStep;
	segment.millisCum+=segment.millisStep;
}



void readCompass()
{

  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(ANGLE_8);                    //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 5);       
  
        // Wait for all bytes to come back
  
  angle8 = Wire.read();               // Read back the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();
  
  angle16 = high_byte;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
}


void loop() {
}
