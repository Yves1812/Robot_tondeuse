// 2018-10 Yves Bonnefont

// Units : position in ticks, speed in ticks/s, bearing in 255/360°, angular speed in 255/360°/ms
// nominal 1 tick every 3ms
// distance 1,5mm / tick
// nominal 350 ticks/s or 52cm/s
// nominal rotation speed 62°/s (assuming radius of wheel to rotation center = 0,5m) 

// to do
// set-up the two I2C connections
// create 0.1s drumbeat and consecutive actions (read decoders, determine adjustments to deliver segment, apply adjustment, ask for next segment when relevant
// clarify communication protocol with MU - MU sends request for x bytes, TP will send current segment cum status:
// * ticks_cum - long
// * millis_cum -long
// * average_bearing - byte
// * current_bearing - byte
// * speed_step - float
// * teta_point float
// * motor_power x4 - byte x4
// check calculation method between floats and ints to avoid stupod roundings

#include <Wire.h>

#define DB_address 0x8
#define COMPASS_address 0x60
#define ANGLE_8  1           // Register to read 8bit angle from

// Sensor reading function header
bool readCompass();
bool readDecoders();

struct segmentOrder{
  int segment_type; //0 in case rotation, 1 in case straight
  long target_ticks;
  int target_bearing;
  int target_speed;
};


class Motor{
  public:
    byte pinforward;
    byte pinbackward;
    byte pinspeed;
    int myspeed;
    byte power;

    Motor(void);
    void Set(int PinForward,int PinBackward,int PinSpeed);
    void Forward();
    void Backward();
    void Stop();
    void SetSpeed();
};


class Rover {
  public:
    Motor FL_motor;
    Motor FR_motor;
    Motor RL_motor;
    Motor RR_motor;
	  long x;
	  long y;
	  int vx;
	  int vy;
	  byte bearing;
	  int teta_point;

    Rover(void);
    void move_rover(void);
};

class SegmentStatus {
  public:
  	long FL_ticks_cum;
  	long FR_ticks_cum;
    long RL_ticks_cum;
    long RR_ticks_cum;
    long ticks_cum;
    int gap_cum;
    unsigned long millis_cum;
    short speed_cum;
  
    short FL_ticks_step;
  	short FR_ticks_step;
  	short RL_ticks_step;
  	short RR_ticks_step;
  	short ticks_step;
    short gap_step;
  	int millis_step;
  	short speed_step;
  	
  	short last_bearing;
    short current_bearing;
    short average_bearing;

    SegmentStatus(void);
    boolean updateStatus();
    void deliverStraightSegment();
    void deliverRotationSegment();
};


// global variabes
unsigned long last_moment=0;
int I2C_buffer[5];
Rover rover;
SegmentStatus segment;
segmentOrder target_move;

//PinForwardR=31
//PinBackwardR=29
//PinSpeedR=23
//sleeptime=1


// Classes member functions
SegmentStatus::SegmentStatus(void){
  this->FL_ticks_cum=0;
  this->FR_ticks_cum=0;
  this->RL_ticks_cum=0;
  this->RR_ticks_cum=0;
  this->ticks_cum=0;
  this->gap_cum=0;
  this->millis_cum=0;
  this->speed_cum=0;

  //Initiate compass   
  this->last_bearing=readCompass();
  this->average_bearing=this->last_bearing;
}

boolean SegmentStatus::updateStatus(){
// reads sensors, updates segment and returns true if ok, false otherwise

  if (readDecoders() && readCompass()) {
    this->FL_ticks_cum+=this->FL_ticks_step;
    this->FR_ticks_cum+=this->FR_ticks_step;
    this->RL_ticks_cum+=this->RL_ticks_step;
    this->RR_ticks_cum+=this->RR_ticks_step;
    this->millis_cum+=this->millis_step;

    if (target_move.segment_type==1){ // this is a straight segment
      this->ticks_step=min(this->FL_ticks_step, this->RL_ticks_step)+min(this->FR_ticks_step, this->RR_ticks_step);
      this->ticks_cum+=this->ticks_step;
      this->speed_step=this->ticks_step/this->millis_step;
      this->speed_cum=this->ticks_cum/this->millis_cum;
      this->average_bearing=((this->current_bearing+this->last_bearing)*this->ticks_step/2+(this->ticks_cum-this->ticks_step)*this->average_bearing)/this->ticks_cum; //shall be in an update status method of segment to for better sync of bearing anf ticks measurment
      this->gap_cum+=min(this->FL_ticks_step, this->RL_ticks_step)-min(this->FR_ticks_step, this->RR_ticks_step);
    }
    return true;
  }
  else
  {
    return false;
  }
}

void SegmentStatus::deliverStraightSegment() {
// would be cleaner to return an array of the 4 target speeds or make this part of rover class with motors being elements of rover
  float LR_PWM_factor, FR_Left_PWM_factor, FR_Right_PWM_factor, scaling_factor;
  int target_speed;

  if (segment.gap_cum>=2 || segment.gap_cum<=-2){
    LR_PWM_factor=1+segment.gap_cum/segment.ticks_cum;
  }
  if (segment.FL_ticks_step-segment.RL_ticks_step!=0){
    FR_Left_PWM_factor=1+(segment.FL_ticks_step-segment.RL_ticks_step)/min(segment.FL_ticks_step, segment.RL_ticks_step); // Slow right side
  }
  if (segment.FR_ticks_step-segment.RR_ticks_step!=0){
    FR_Right_PWM_factor=1+(segment.FR_ticks_step-segment.RR_ticks_step)/min(segment.FR_ticks_step, segment.RR_ticks_step); // Slow right side
  }
  rover.FL_motor.myspeed*=LR_PWM_factor*FR_Left_PWM_factor;
  rover.FR_motor.myspeed*=FR_Right_PWM_factor;

  if (target_move.target_ticks-segment.ticks_cum<100){
    target_speed=target_move.target_speed/4;
  }
  else
  {
    target_speed=target_move.target_speed/4;
  }

  scaling_factor=max(max(rover.FL_motor.myspeed, rover.FL_motor.myspeed), max(rover.FL_motor.myspeed,rover.FL_motor.myspeed))/target_speed;
  if (scaling_factor>0.8){
    if (rover.FL_motor.myspeed!=0){rover.FL_motor.myspeed*=1/scaling_factor;} else {rover.FL_motor.myspeed=1;}        
    if (rover.FR_motor.myspeed!=0){rover.FR_motor.myspeed*=1/scaling_factor;} else {rover.FR_motor.myspeed=1;}
    if (rover.RL_motor.myspeed!=0){rover.RL_motor.myspeed*=1/scaling_factor;} else {rover.RL_motor.myspeed=1;}        
    if (rover.RR_motor.myspeed!=0){rover.RR_motor.myspeed*=1/scaling_factor;} else {rover.RR_motor.myspeed=1;}        
  }
  else {
    if (rover.FL_motor.myspeed!=0){rover.FL_motor.myspeed*=1.2;} else {rover.FL_motor.myspeed=1;}        
    if (rover.FR_motor.myspeed!=0){rover.FR_motor.myspeed*=1.2;} else {rover.FR_motor.myspeed=1;}        
    if (rover.RL_motor.myspeed!=0){rover.RL_motor.myspeed*=1.2;} else {rover.RL_motor.myspeed=1;}        
    if (rover.RR_motor.myspeed!=0){rover.RR_motor.myspeed*=1.2;} else {rover.RR_motor.myspeed=1;}        
  }
}

void SegmentStatus::deliverRotationSegment() {
  // done with compass
  // could be done with ticks by converting the rotation angle into ticks based on bearing radius = 0.5m and nominal speed = 0.52m/s, one ticl = 1,5mm
  // all this to be checked on final assembly
  // manage turn right or left to go faster
  int gap_to_target_bearing;
  
  gap_to_target_bearing=abs(this->current_bearing-target_move.target_bearing);
  if (gap_to_target_bearing>5)
  {
    rover.FL_motor.myspeed=128; 
    rover.FR_motor.myspeed=-128;        
    rover.RL_motor.myspeed=128;        
    rover.RR_motor.myspeed=-128;        
  }
  else {
    rover.FL_motor.myspeed=64;
    rover.FR_motor.myspeed=-64;
    rover.RL_motor.myspeed=64;
    rover.RR_motor.myspeed=-64;
  }
}

Motor::Motor(void)
{
  this->pinforward=0;
  this->pinbackward=0;
  this->pinspeed=0;
  this->myspeed=0;
  
}

void Motor::set(int PinForward,int PinBackward, int PinSpeed){
  this->pinforward=PinForward;
  this->pinbackward=PinBackward;
  this->pinspeed=PinSpeed;
  this->myspeed=0;
  digitalWrite(this->pinforward, OUTPUT);
  digitalWrite(this->pinbackward, OUTPUT);
  digitalWrite(this->pinspeed, OUTPUT);
  analogWrite(this->pinspeed, this->myspeed);
}

void Motor::SetSpeed(){
  if (this->myspeed >0){
    digitalWrite(this->pinforward, HIGH);
    digitalWrite(this->pinbackward, LOW);
    analogWrite(this->pinspeed, this->myspeed);
  }
  if (this->myspeed == 0){
    digitalWrite(this->pinforward, LOW);
    digitalWrite(this->pinbackward, LOW);
  }
  if (this->myspeed <0){
    digitalWrite(this->pinforward, LOW);
    digitalWrite(this->pinbackward, HIGH);
    analogWrite(this->pinspeed, -this->myspeed);
  }
}

void Motor::Stop(){
  digitalWrite(this->pinforward, LOW);
  digitalWrite(this->pinbackward, LOW);
}

Rover::Rover(void){
  this->FL_motor.set(31,29,23);
  this->FR_motor.set(31,29,23);
  this->RL_motor.set(31,29,23);
  this->RR_motor.set(31,29,23);
}

void Rover::move_rover(void){
  this->FL_motor.SetSpeed();
  this->FR_motor.SetSpeed();
  this->RL_motor.SetSpeed();
  this->RR_motor.SetSpeed();
}

void setup() {
// Set Serial communication for debugging purpose
  delay(1000);
  Serial.begin(9600);  // start serial for output
  Serial.print("Serial bus set-up");
// I2C bus set-up
   Wire.begin();                      // join Sensors i2c bus as master, no address needed
//   Wire1.begin(0x16);                // join MU i2c bus as a slave with address 0x16 (0-7 eserved) 
//   Wire1.onRequest(requestEvent);   // register event on MU bus

// Set time interrupt for drumbeat
// settimer_interrupt 1s drumBeat()
   Serial.println("Wire started");
}

// Interrupt service routines
// I2C write data in response to IMU request
void requestEvent() {
// * ticks_cum - long
// * millis_cum -long
// * average_bearing - byte
// * current_bearing - byte
// * speed_step - float
// * teta_point float
// * motor_power x4 - byte x4

// 20 bytes to be sent
Wire1.write(segment.ticks_cum); //4 bytes
Wire1.write(segment.millis_cum); // 4 bytes
Wire1.write(segment.average_bearing); // 2 bytes
Wire1.write(segment.current_bearing); // 2 bytes
Wire1.write(segment.speed_step); // 2 bytes
//Wire1.write(segment.teta_point); // 2 bytes
Wire1.write(rover.FL_motor.power); //1 byte
Wire1.write(rover.FR_motor.power); //1 byte
Wire1.write(rover.RL_motor.power); //1 byte
Wire1.write(rover.RR_motor.power); //1 byte
}

bool drumBeat() {
  segment.updateStatus();
  if (target_move.segment_type == 1){
    segment.deliverStraightSegment();
  }
  if (target_move.segment_type == 0){
    segment.deliverRotationSegment();
  }
  rover.move_rover();
//  TO BE DEVELOPPED
}

bool readDecoders() {
   int retries=0;
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

boolean readCompass()
// returns current bearing as a byte or -1 if error while reading
{
  int retries=0;
  unsigned long now;

  Wire.beginTransmission(COMPASS_address);//starts communication with CMPS12
  Wire.write(ANGLE_8);                    //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 1 byte from the CMPS12
  // this will give us the 8 bit bearing
   while (retries<4) { //checks requested bytes were received 3 retries max
      now=millis();
      Wire.requestFrom(COMPASS_address, 1);
      while (Wire.available()<1 && millis()-now < 100); // need slave to send no less than requested
      if (Wire.available()== 1) {
        segment.last_bearing=segment.current_bearing;
        segment.current_bearing = Wire.read();        // Read the 1 bytes
        return true;
      }
      retries++;
   }
   return false;
}


void loop() {
   Serial.println(last_moment);
   if (millis()-last_moment>100){ // for testing purpose
     last_moment=millis();
     Serial.print(readDecoders());
     Serial.println(segment.FL_ticks_step);
     Serial.println(segment.FR_ticks_step);
     Serial.println(segment.RL_ticks_step);
     Serial.println(segment.RR_ticks_step);
   }
}
