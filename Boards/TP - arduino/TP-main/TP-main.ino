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
	int FL_ticks_cum=0;
	int FR_ticks_cum=0;
	int RL_ticks_cum=0;
	int RR_ticks_cum=0;
	int ticks_cum=0;
    int gap_cum=0;
	int millis_cum=0;

    int FL_ticks_step;
	int FR_ticks_step;
	int RL_ticks_step;
	int RR_ticks_step;
	int ticks_step;
    int gap_step;
	int millis_step;
	int speed_step;
	
	int last_bearing;
    int current_bearing;
}

int I2C_buffer[5];
roverType rover;
segmentStatus segment;
segmentOrder target_move; next_move;

void setup() {
   Wire.begin();        // join i2c bus (address optional for master)
   // need to set-up the two I2C connection
   Serial.begin(9600);  // start serial for output
   
   last_bearing=readCompass();
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
  segment.FL_ticksCum+=segment.FL_ticksStep;
  segment.FR_ticksCum+=segment.FR_ticksStep;
  segment.RL_ticksCum+=segment.RL_ticksStep;
  segment.RR_ticksCum+=segment.RR_ticksStep;
  segment.millisCum+=segment.millisStep;
  segment.cum_gap+=min(segment.FL_ticks_step, segment.RL_ticks_step)-min(segment.FR_ticks_step, segment.RR_ticks_step);
  segment.ticks_step=min(segment.FL_ticks_step, segment.RL_ticks_step)+min(segment.FR_ticks_step, segment.RR_ticks_step);
  segment.ticks_cum+=segment.ticks_step;
  segment.speed_step=segment.ticks_step/segment.millis_step;
  
  if (segment.cum_gap>=2 || segment.cum_gap<=-2){
    LR_PWM_factor=1+segment.cum_gap/segment.ticks_cum;
  }
  if (segment.FL_ticks_step-segment.RL_ticks_step!=0){
    FRLeft_PWM_factor=1+(segment.FL_ticks_step-segment.RL_ticks_step)/min(segment.FL_ticks_step, segment.RL_ticks_step); // Slow right side
  }
  if (segment.FR_ticks_step-segment.RR_ticks_step!=0){
    FRRight_PWM_factor=1+(segment.FR_ticks_step-segment.Rr_ticks_step)/min(segment.FR_ticks_step, segment.RR_ticks_step); // Slow right side
  }
  
  FL_PWM=FL_PWM*LR_PWM_factor*FRLeft_PWM_factor;
  FR_PWM=FR_PWM*FRRight_PWM_factor;
  
  speed_adjust=max(
  
}



int readCompass()
{
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(ANGLE_8);                    //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 1);
  int angle8 = (int) Wire.read();               // Read back the 5 bytes
  return angle8;
}


void loop() {
}
