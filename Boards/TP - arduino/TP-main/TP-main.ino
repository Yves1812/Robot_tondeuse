// 2018-10 Yves Bonnefont

// Units : position in ticks, speed in ticks/s, bearing in 255/360°, angular speed in 255/360°/ms
// distance 1,5mm / tick
// nominal 350 ticks/s ou 52cm/s

#include <Wire.h>

#define DB_address 0x8
#define COMPASS_address 0x60
#define ANGLE_8  1           // Register to read 8bit angle from

struct roverType {
	long x;
	long y;
	int vx;
	int vy;
	byte bearing;
	int teta_point;
};

struct segmentOrder{
  int segment_type;
  long target_ticks;
  int target_bearing;
  int target_speed;
};

struct segmentStatus {
	long FL_ticks_cum=0;
	long FR_ticks_cum=0;
	long RL_ticks_cum=0;
	long RR_ticks_cum=0;
	long ticks_cum=0;
  int gap_cum=0;
	unsigned long millis_cum=0;
  float speed_cum=0;

  int FL_ticks_step;
	int FR_ticks_step;
	int RL_ticks_step;
	int RR_ticks_step;
	int ticks_step;
  int gap_step;
	int millis_step;
	float speed_step;
	
	int last_bearing;
  int current_bearing;
};

class Motor
{
  public:
    byte pinforward;
    byte pinbackward;
    byte pinspeed;
    int myspeed;
    Motor(int PinForward,int PinBackward,int PinSpeed);
    void Forward();
    void Backward();
    void Stop();
    void SetSpeed();
};

Motor::Motor(int PinForward,int PinBackward, int PinSpeed){
  this->pinforward=PinForward;
  this->pinbackward=PinBackward;
  this->pinspeed=PinSpeed;
  this->myspeed=0;
  digitalWrite(this->pinforward, OUTPUT);
  digitalWrite(this->pinbackward, OUTPUT);
  digitalWrite(this->pinspeed, OUTPUT);
  analogWrite(this->pinspeed, this->myspeed);
}

void Motor::Forward(){
  digitalWrite(this->pinforward, HIGH);
}

void Motor::Backward(){
  digitalWrite(this->pinbackward, HIGH);
}

void Motor::SetSpeed(){
  analogWrite(this->pinspeed, this->myspeed);
}

void Motor::Stop(){
  digitalWrite(this->pinforward, LOW);
  digitalWrite(this->pinbackward, LOW);
}

//PinForwardR=31
//PinBackwardR=29
//PinSpeedR=23
//sleeptime=1

Motor FL_motor(31,29,23);
Motor FR_motor(31,29,23);
Motor RL_motor(31,29,23);
Motor RR_motor(31,29,23);

int I2C_buffer[5];
roverType rover;
segmentStatus segment;
segmentOrder target_move;

void setup() {
   Wire.begin();        // join i2c bus (address optional for master)
   // need to set-up the two I2C connection
   Serial.begin(9600);  // start serial for output
   
   segment.last_bearing=readCompass();
}

bool readDecoders() {
   byte retries=0;
   unsigned long now;
   
   while (retries<4) { //checks 5 bytes were received 3 retries max
      now=millis();
      Wire.requestFrom(DB_address, 5);    // request 5 bytes from slave device DB_board
      while (Wire.available()<5 && millis()-now < 100); // need slave to send no less than requested
      if (Wire.available()== 5) {
         segment.FL_ticks_step=((int) Wire.read())-127;
         segment.FR_ticks_step=((int) Wire.read())-127;
         segment.RL_ticks_step=((int) Wire.read())-127;
         segment.RR_ticks_step=((int) Wire.read())-127;
         segment.millis_step=(int) Wire.read();
         return true;
      }
      retries++;
   }
   return false;
}

int deliverStraightSegment() {
  float LR_PWM_factor, FR_Left_PWM_factor, FR_Right_PWM_factor, scaling_factor;
   
  segment.FL_ticks_cum+=segment.FL_ticks_step;
  segment.FR_ticks_cum+=segment.FR_ticks_step;
  segment.RL_ticks_cum+=segment.RL_ticks_step;
  segment.RR_ticks_cum+=segment.RR_ticks_step;
  segment.ticks_step=min(segment.FL_ticks_step, segment.RL_ticks_step)+min(segment.FR_ticks_step, segment.RR_ticks_step);
  segment.ticks_cum+=segment.ticks_step;

  if (segments.tick_cum<target_move.target_ticks):
  {
    segment.millis_cum+=segment.millis_step;
    segment.gap_cum+=min(segment.FL_ticks_step, segment.RL_ticks_step)-min(segment.FR_ticks_step, segment.RR_ticks_step);
    segment.speed_step=segment.ticks_step/segment.millis_step;
    segment.speed_cum=segment.ticks_cum/segment.millis_cum;
    
    if (segment.gap_cum>=2 || segment.gap_cum<=-2){
      LR_PWM_factor=1+segment.gap_cum/segment.ticks_cum;
    }
    if (segment.FL_ticks_step-segment.RL_ticks_step!=0){
      FR_Left_PWM_factor=1+(segment.FL_ticks_step-segment.RL_ticks_step)/min(segment.FL_ticks_step, segment.RL_ticks_step); // Slow right side
    }
    if (segment.FR_ticks_step-segment.RR_ticks_step!=0){
      FR_Right_PWM_factor=1+(segment.FR_ticks_step-segment.RR_ticks_step)/min(segment.FR_ticks_step, segment.RR_ticks_step); // Slow right side
    }
    FL_motor.myspeed*=LR_PWM_factor*FR_Left_PWM_factor;
    FR_motor.myspeed*=FR_Right_PWM_factor;
  
    scaling_factor=max(max(FL_motor.myspeed, FL_motor.myspeed), max(FL_motor.myspeed,FL_motor.myspeed))/target_move.target_speed;
    if (scaling_factor>0.8){
      if (FL_motor.myspeed!=0){FL_motor.myspeed*=1/scaling_factor;) else {FL_motor.myspeed=1;}        
      if (FL_motor.myspeed!=0){FR_motor.myspeed*=1/scaling_factor;) else {FR_motor.myspeed=1;}
      if (FL_motor.myspeed!=0){RL_motor.myspeed*=1/scaling_factor;) else {RL_motor.myspeed=1;}        
      if (FL_motor.myspeed!=0){RR_motor.myspeed*=1/scaling_factor;) else {RR_motor.myspeed=1;}        
    }
    else {
      if (FL_motor.myspeed!=0){FL_motor.myspeed*=1.2;) else {FL_motor.myspeed=1;}        
      if (FR_motor.myspeed!=0){FR_motor.myspeed*=1.2;) else {FR_motor.myspeed=1;}        
      if (RL_motor.myspeed!=0){RL_motor.myspeed*=1.2;) else {RL_motor.myspeed=1;}        
      if (RR_motor.myspeed!=0){RR_motor.myspeed*=1.2;) else {RR_motor.myspeed=1;}        
    }
    return target_move.target_ticks-segments.tick_cum;
  }
  else
  {
    FL_motor.myspeed=0;
    FL_motor.myspeed=0;
    FL_motor.myspeed=0;
    FL_motor.myspeed=0;
    return 0;
  }
}


void deliverRotationSegment() {
  float LR_PWM_factor, FR_Left_PWM_factor, FR_Right_PWM_factor, scaling_factor;

  segment.FL_ticks_cum+=segment.FL_ticks_step;
  segment.FR_ticks_cum+=segment.FR_ticks_step;
  segment.RL_ticks_cum+=segment.RL_ticks_step;
  segment.RR_ticks_cum+=segment.RR_ticks_step;
  segment.millis_cum+=segment.millis_step;
  segment.gap_cum+=min(segment.FL_ticks_step, segment.RL_ticks_step)-min(segment.FR_ticks_step, segment.RR_ticks_step);
  segment.ticks_step=min(segment.FL_ticks_step, segment.RL_ticks_step)+min(segment.FR_ticks_step, segment.RR_ticks_step);
  segment.ticks_cum+=segment.ticks_step;
  segment.speed_step=segment.ticks_step/segment.millis_step;
  segment.speed_cum=segment.ticks_cum/segment.millis_cum;
  
  if (segment.gap_cum>=2 || segment.gap_cum<=-2){
    LR_PWM_factor=1+segment.gap_cum/segment.ticks_cum;
  }
  if (segment.FL_ticks_step-segment.RL_ticks_step!=0){
    FR_Left_PWM_factor=1+(segment.FL_ticks_step-segment.RL_ticks_step)/min(segment.FL_ticks_step, segment.RL_ticks_step); // Slow right side
  }
  if (segment.FR_ticks_step-segment.RR_ticks_step!=0){
    FR_Right_PWM_factor=1+(segment.FR_ticks_step-segment.RR_ticks_step)/min(segment.FR_ticks_step, segment.RR_ticks_step); // Slow right side
  }
  FL_motor.myspeed*=LR_PWM_factor*FR_Left_PWM_factor;
  FR_motor.myspeed*=FR_Right_PWM_factor;

  scaling_factor=max(max(FL_motor.myspeed, FL_motor.myspeed), max(FL_motor.myspeed,FL_motor.myspeed))/target_move.target_speed;
  if (scaling_factor>0.8){
    FL_motor.myspeed*=1/scaling_factor;
    FL_motor.myspeed*=1/scaling_factor;
    FL_motor.myspeed*=1/scaling_factor;
    FL_motor.myspeed*=1/scaling_factor;
  }
  else {
    FL_motor.myspeed*=1.2;
    FL_motor.myspeed*=1.2;
    FL_motor.myspeed*=1.2;
    FL_motor.myspeed*=1.2;
  }
}

int readCompass()
{
  Wire.beginTransmission(COMPASS_address);  //starts communication with CMPS12
  Wire.write(ANGLE_8);                    //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(COMPASS_address, 1);
  int angle8 = (int) Wire.read();               // Read back the 5 bytes
  return angle8;
}


void loop() {
  FL_motor.SetSpeed();
  FR_motor.SetSpeed();
  RL_motor.SetSpeed();
  RR_motor.SetSpeed();
}
