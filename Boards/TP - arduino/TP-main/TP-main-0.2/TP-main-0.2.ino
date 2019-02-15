// 2018-10 Yves Bonnefont

// Units : position in ticks, speed + or - 255/(nominal ticks/s), bearing in 255/360°, angular speed in 255/360°/ms
// nominal 1 tick every 3ms
// distance 1,5mm / tick
// nominal 350 ticks/s or 52cm/s
// nominal rotation speed 49°/s or 35 bits/s or 0,035 bit/millis (assuming radius of wheel to rotation center = 0,6m) 

// to do
/// * manage emergency stops on rotation segments
/// * manage speed adjustments on rotation segments
/// * Check pins forward and backward

// SPI bus data formats
// * ticks_cum - long
// * millis_cum -long
// * average_bearing - byte
// * current_bearing - byte
// * speed_step - byte in SPI [0-255] = [-350,350] as int (ticks/s) in the rest of the program
// * teta_point - byte
// * motor_power x4 - byte x4
// check calculation method between floats and ints to avoid stupid roundings

#include <SPI.h>
#include <Wire.h>


//Physical pins of motors
// Need to check after connection that pins are allocated to relevant motor and rotation directions are consistent
#define PinForwardFL 22
#define PinBackwardFL 24
#define PinSpeedFL 3

#define PinForwardFR 25
#define PinBackwardFR 23
#define PinSpeedFR 2

#define PinForwardRL 26
#define PinBackwardRL 28
#define PinSpeedRL 5

#define PinForwardRR 29
#define PinBackwardRR 27
#define PinSpeedRR 4

// Compass pin
#define ANGLE_8  18           // Register to read 8 bits angle from compass
// serial 1 RX = 19
// serial 1 Tx = 18

// SPI Slave Select pins
#define SS_DECODER_BOARD 22

// Constant
#define MAX_SPEED 350.0 // ticks per second
#define MAX_ROTATION 35.0 // bit per second

// I2C address and registers
// SCL 21
// SDA 20
#define SLAVE_ADDRESS 0x16
#define SEGMENT_REGISTER 10
#define MOTORS_REGISTER 20
#define ROUTING_REGISTER 30


struct segmentOrder{
  byte segment_type; //0 in case rotation, 1 in case straight
  unsigned int segment_id;
  long target_ticks;
  byte target_bearing;
  int target_speed; // Can vary between +350 and -350 converted from 1 byte SPI
};

class Motor{
  public:
    byte pinforward;
    byte pinbackward;
    byte pinspeed;
    int myspeed;
    byte power;

    Motor(void);
    void set(int PinForward,int PinBackward,int PinSpeed);
    void stop();
    void setSpeed();
};

class Rover {
  public:
    Motor FL_motor;
    Motor FR_motor;
    Motor RL_motor;
    Motor RR_motor;
//	  long x;
//	  long y;
//	  int vx;
//	  int vy;
//	  byte bearing;
//	  byte teta_point;

    Rover(void);
    void begin(void);
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
    int speed_cum;
  
    short FL_ticks_step;
  	short FR_ticks_step;
  	short RL_ticks_step;
  	short RR_ticks_step;
  	short ticks_step;
    short gap_step;
  	int millis_step;
  	int speed_step;
  	
  	byte last_bearing;
    byte current_bearing;
    byte average_bearing;

    SegmentStatus(void);
    void begin(void);
    boolean updateStatus(void);
    boolean readCompass(void);
    boolean readDecoders(void);
    void deliverStraightSegment(void);
    void deliverRotationSegment(void);
};

// *************** global variabes ****************************************************************************** //
// ************************************************************************************************************** //
unsigned long last_moment=0;
volatile int I2C_buffer[5];
volatile boolean seg_completed=true, next_seg_available=false; // seg completed initialized to true to enable initial seg_order load
volatile Rover rover;
volatile SegmentStatus segment;
volatile segmentOrder current_move;
volatile segmentOrder next_move;
volatile byte I2Cregister;
// *************************************************************************************************************** //
// *************************************************************************************************************** //

// Classes member functions
SegmentStatus::SegmentStatus(void){
  FL_ticks_cum=0;
  FR_ticks_cum=0;
  RL_ticks_cum=0;
  RR_ticks_cum=0;
  ticks_cum=0;
  gap_cum=0;
  millis_cum=0;
  speed_cum=0;
}

void SegmentStatus::begin(void){
  //Initiate compass   
    FL_ticks_cum=0;
    FR_ticks_cum=0;
    RL_ticks_cum=0;
    RR_ticks_cum=0;
    ticks_cum=0;
    gap_cum=0;
    millis_cum=0;
    speed_cum=0;
  
    FL_ticks_step=0;
    FR_ticks_step=0;
    RL_ticks_step=0;
    RR_ticks_step=0;
    ticks_step=0;
    gap_step=0;
    millis_step=0;
    speed_step=0;
    
    last_bearing=0;
    readCompass();
    average_bearing=last_bearing;
}

boolean SegmentStatus::updateStatus(){
// reads sensors, updates segment and returns true if ok, false otherwise
  if (readDecoders() && readCompass()) {
    FL_ticks_cum+=FL_ticks_step;
    FR_ticks_cum+=FR_ticks_step;
    RL_ticks_cum+=RL_ticks_step;
    RR_ticks_cum+=RR_ticks_step;
    millis_cum+=millis_step;

    if (current_move.segment_type==1){ // this is a straight segment
      ticks_step=min(min(FL_ticks_step, RL_ticks_step),min(FR_ticks_step, RR_ticks_step));
      ticks_cum+=ticks_step;
      speed_step=ticks_step*1000/millis_step;
      speed_cum=ticks_cum*1000/millis_cum;
      average_bearing=((current_bearing+last_bearing)*ticks_step/2+(ticks_cum-ticks_step)*average_bearing)/ticks_cum; //average bearing since start of segment
      gap_cum+=min(FL_ticks_step, RL_ticks_step)-min(FR_ticks_step, RR_ticks_step);
    }
    if (current_move.segment_type==1){
      if (current_move.target_ticks-segment.ticks_cum<10){
        seg_completed=true;
      }
    }
    if (current_move.segment_type==0){
      if (abs(current_bearing-current_move.target_bearing)<2){
        seg_completed=true;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

void SegmentStatus::deliverStraightSegment() {
  float LR_PWM_factor=1.0, FR_Left_PWM_factor=1.0, FR_Right_PWM_factor=1.0, scaling_factor=1.0;
  int target_speed;

  if (segment.gap_cum>=2 || segment.gap_cum<=-2){ // There is a gap between left slowest wheel and right slowest wheel
    LR_PWM_factor=1+segment.gap_cum/segment.ticks_cum; // Left-Right factor to be adjusted to correct the gap
  }
  if (segment.FL_ticks_step-segment.RL_ticks_step!=0){ // There is a gap of speed between one wheel and the other on the Left side
    FR_Left_PWM_factor=1+(segment.FL_ticks_step-segment.RL_ticks_step)/min(segment.FL_ticks_step, segment.RL_ticks_step); // Adjust Front Rear balance on the Left side
  }
  if (segment.FR_ticks_step-segment.RR_ticks_step!=0){ // There is a gap of speed between one wheel and the other on the Right side 
    FR_Right_PWM_factor=1+(segment.FR_ticks_step-segment.RR_ticks_step)/min(segment.FR_ticks_step, segment.RR_ticks_step); // // Adjust Front Rear balance on the Right side
  }

  // Apply adjustment factors Rear right arbitrary used as the reference - alternative is to use the sloest wheel as a reference (to be explored in Vn+1)
  // Adjust left side for difference with right side
  rover.FL_motor.myspeed*=LR_PWM_factor;
  rover.RL_motor.myspeed*=LR_PWM_factor;
  // adjust each side for Front Rear Balance
  rover.FL_motor.myspeed*=FR_Left_PWM_factor;
  rover.FR_motor.myspeed*=FR_Right_PWM_factor;

  // Scale overall speed to reconverge towards desired target.speed
  if (current_move.target_ticks-segment.ticks_cum<100){
    target_speed=current_move.target_speed/4; // If aproaching end of segment, speed is divided by 4 vs target speed
  }
  else
  {
    target_speed=current_move.target_speed;
  }
  // added max(1,...) to avoid errors when all motor speeds are 0 - this is the case at start!
  scaling_factor=target_speed/(max(1,max(max(rover.FL_motor.myspeed, rover.FR_motor.myspeed), max(rover.RL_motor.myspeed,rover.RR_motor.myspeed))));
  // Cap speed variation to +/- 20%
  if (scaling_factor <0.8) {scaling_factor=0.8;}
  if (scaling_factor > 1.2) {scaling_factor = 1.2;}
  if (rover.FL_motor.myspeed!=0){rover.FL_motor.myspeed*=scaling_factor;} else {rover.FL_motor.myspeed=0.01*target_speed;} // if a speed had gone to zero, get it back non null unless target speed is zero!        
  if (rover.FR_motor.myspeed!=0){rover.FR_motor.myspeed*=scaling_factor;} else {rover.FR_motor.myspeed=0.01*target_speed;}
  if (rover.RL_motor.myspeed!=0){rover.RL_motor.myspeed*=scaling_factor;} else {rover.RL_motor.myspeed=0.01*target_speed;}        
  if (rover.RR_motor.myspeed!=0){rover.RR_motor.myspeed*=scaling_factor;} else {rover.RR_motor.myspeed=0.01*target_speed;}        
}

void SegmentStatus::deliverRotationSegment() {
  // done with compass
  // could be done with ticks by converting the rotation angle into ticks based on bearing radius = 0.5m and nominal speed = 0.52m/s, one ticl = 1,5mm
  // all this to be checked on final assembly
  // manage turn right or left to go faster
  int gap_to_target_bearing;
  
  gap_to_target_bearing=abs(current_bearing-current_move.target_bearing); // if target position is passed, will reaccelerate and do à 360°
  if (gap_to_target_bearing>5)
  {
	// Rotate @ half speed
    rover.FL_motor.myspeed=175; 
    rover.FR_motor.myspeed=-175;        
    rover.RL_motor.myspeed=175;        
    rover.RR_motor.myspeed=-175;        
  }
  else {
	// Rotate @ 10% speed getting close to target angle
    rover.FL_motor.myspeed=35;
    rover.FR_motor.myspeed=-35;
    rover.RL_motor.myspeed=35;
    rover.RR_motor.myspeed=-35;
  }
}

bool SegmentStatus::readDecoders(){
// Need error management
  SPI.beginTransaction (SPISettings (2000000, MSBFIRST, SPI_MODE0));  // 2 MHz clock
  // enable Slave Select
  digitalWrite(SS_DECODER_BOARD, LOW);
  // Collect the 5 bytes
  SPI.transfer(255); // 's' like Start to trigger matching on slave
  delayMicroseconds (500); // wait for the latching to complete
  //read the 5 bytes expected - decoders are set to 127 for 0 ticks
  FL_ticks_step=((int) SPI.transfer(1))-127;
  delayMicroseconds (5); // wait for the latching to complete
  FR_ticks_step=((int) SPI.transfer(2))-127;
  delayMicroseconds (5); // wait for the latching to complete
  RL_ticks_step=((int) SPI.transfer(3))-127;
  delayMicroseconds (5); // wait for the latching to complete
  RR_ticks_step=((int) SPI.transfer(4))-127;
  delayMicroseconds (5); // wait for the latching to complete
  millis_step=(int) SPI.transfer(0);
  // disable Slave Select
  digitalWrite(SS, HIGH);
  SPI.endTransaction ();
  
  return true;
}

boolean SegmentStatus::readCompass(){
// returns current bearing as a byte or -1 if error while reading
  int retries=0;
  unsigned long now;
 
  // Request 1 byte from the CMPS12
  // this will give us the 8 bit bearing
   while (retries<4) { //checks requested bytes were received 3 retries max
      now=millis();
      Serial1.write(ANGLE_8);  // Request and read 8 bit angle
      while (Serial1.available()<1 && millis()-now < 100); // need slave to send no less than requested
      if (Serial1.available()>0) {
        last_bearing=current_bearing;
        current_bearing = Serial1.read();        // Read the 1 bytes
        return true;
      }
      retries++;
   }
   return false;
}


Motor::Motor(void){
  pinforward=0;
  pinbackward=0;
  pinspeed=0;
  myspeed=0; 
}

void Motor::set(int PinForward,int PinBackward, int PinSpeed){
  pinforward=PinForward;
  pinbackward=PinBackward;
  pinspeed=PinSpeed;
  myspeed=0;
  pinMode(pinforward, OUTPUT);
  pinMode(pinbackward, OUTPUT);
  pinMode(pinspeed, OUTPUT);
  analogWrite(pinspeed, myspeed);
}

void Motor::setSpeed(){
  if (myspeed >0){
    digitalWrite(pinforward, HIGH);
    digitalWrite(pinbackward, LOW);
    analogWrite(pinspeed, int(myspeed/MAX_SPEED*255));
  }
  if (myspeed == 0){
    digitalWrite(pinforward, LOW);
    digitalWrite(pinbackward, LOW);
  }
  if (myspeed <0){
    digitalWrite(pinforward, LOW);
    digitalWrite(pinbackward, HIGH);
    analogWrite(pinspeed, int(-myspeed/MAX_SPEED*255));
  }
}

void Motor::stop(){
  digitalWrite(pinforward, LOW);
  digitalWrite(pinbackward, LOW);
}

Rover::Rover(void){
  last_moment=0;
}
void Rover::begin(void){
  FL_motor.set(PinForwardFL,PinBackwardFL,PinSpeedFL);
  FR_motor.set(PinForwardFR,PinBackwardFR,PinSpeedFR);
  RL_motor.set(PinForwardRL,PinBackwardRL,PinSpeedRL);
  RR_motor.set(PinForwardRR,PinBackwardRR,PinSpeedRR);
}

void Rover::move_rover(void){
    FL_motor.setSpeed();
    FR_motor.setSpeed();
    RL_motor.setSpeed();
    RR_motor.setSpeed();
}

void setup() {
// Set Serial communication for debugging purpose
  delay(100);
  Serial.begin(9600);  
  Serial1.begin(9600, SERIAL_8N2);
  Serial.println("Booting up ...");
  Serial.println("Serial bus initiated");

  segment.begin();
  rover.begin();
  current_move.segment_id=0; //0 means no initial order has been received

// I2C bus set-up
  Wire.begin(SLAVE_ADDRESS);               // join MU i2c bus as a slave with address 0x16 (0-7 reserved) 
  Wire.onRequest(requestEvent);   // register event on MU bus
  Wire.onReceive(receiveEvent);
  Serial.println("I2C Main Unit bus initiated as a slave");
  I2Cregister=0;
  
// SPI bus initiated
  digitalWrite(SS_DECODER_BOARD, HIGH);  // ensure SS for DECODER_BOARD stays high for now
  // Put SCK, MOSI, SS pins into output mode
  // also put SCK, MOSI into LOW state, and SS into HIGH state.
  // Then put SPI hardware into Master mode and turn SPI on
  SPI.begin ();
  // Slow down the master a bit
  SPI.setClockDivider(SPI_CLOCK_DIV8);

// To be commented out for test purpose
//  while (not next_seg_available) {
//    delay(100);
//  } // Wait for first order_segment to be sent
//  current_move.segment_type=next_move.segment_type; //0 in case rotation, 1 in case straight
//  current_move.segment_id=next_move.segment_id;
//  current_move.target_ticks=next_move.target_ticks;
//  current_move.target_bearing=next_move.target_bearing;
//  current_move.target_speed=next_move.target_speed;
//  next_seg_available=false;
//  seg_completed=false;

// to be used for test only
//  current_move.segment_type=1; //0 in case rotation, 1 in case straight
//  current_move.segment_id=1;
//  current_move.target_ticks=10000;
//  current_move.target_bearing=90;
//  current_move.target_speed=100; // Can vary between +350 and -350 converted from 1 byte SPI

  segment.begin(); // requires compass to be connected (initial compass read)
}


// Interrupt service routines

void receiveEvent(int howMany) {
  //segment_type 1 byte - 0 in case rotation, 1 in case straight
  //long target_ticks 4 bytes
  //target_bearing 1 byte
  //target_speed 1 byte
  
  byte buff[4];
  long ticks=0;

  if (I2Cregister==0 && howMany == 1){
    I2Cregister=Wire.read();
//    Serial.print("I2Cregister:");
//    Serial.println(I2Cregister);
  }
  else{
    if (howMany == 10){
      I2Cregister=Wire.read();
//      Serial.print("I2Cregister:");
//      Serial.println(I2Cregister);

      if (I2Cregister == ROUTING_REGISTER){
        next_move.segment_type = Wire.read();
    
        buff[0]=Wire.read();
        buff[1]=Wire.read();
        next_move.segment_id=word(buff[0],buff[1]);
//        Serial.print("id:");
//        Serial.println(next_move.segment_id);

        buff[0]=Wire.read();
        buff[1]=Wire.read();
        buff[2]=Wire.read();
        buff[3]=Wire.read();
//        Serial.print(buff[0]);
//        Serial.print(buff[1]);
//        Serial.print(buff[2]);
//        Serial.print(buff[3]);
        
        ticks+=(long)buff[0]<<24;
        ticks+=(long)buff[1]<<16;
        ticks+=(long)buff[2]<<8;
        ticks+=(long)buff[3];
        next_move.target_ticks = ticks;
//        Serial.print("ticks:");
//        Serial.println(ticks);
    
        next_move.target_bearing = Wire.read();
        // convert byte received into a +/- MAX_SPEED value
        next_move.target_speed = int((Wire.read()-127)*2*MAX_SPEED/256);
//        Serial.print("target_bearing:");
//        Serial.println(next_move.target_bearing);
//        Serial.print("target_speed:");
//        Serial.println(next_move.target_speed);
        
        next_seg_available=true;
        I2Cregister=0;
      }
      else{
        Serial.println("Error receiving data, received 10 bytes but unknown register");
      }
    }
    else{
      Serial.println("Error receiving data, received unrecognized number of bytes");
    }
  }
}

void requestEvent() {
  // I2C write data in response to IMU request
  // * ticks_cum - long
  // * millis_cum -long
  // * average_bearing - byte
  // * current_bearing - byte
  // * speed_step - float
  // * teta_point float
  // * motor_power x4 - byte x4
  byte buf[14];
  int i;
  
  // 14 bytes to be sent
  if (I2Cregister==SEGMENT_REGISTER){
    // 14 bytes to be sent
    buf[1]=current_move.segment_id & 0xFF;// //sizeof(current_move.segment_id)); // 2 bytes
    buf[0]=(current_move.segment_id >> 8 )& 0xFF;
    buf[5]=segment.ticks_cum & 0xFF; //sizeof(segment.ticks_cum)); // 4 bytes
    buf[4]=(segment.ticks_cum >>8 ) & 0xFF;
    buf[3]=(segment.ticks_cum >>16 ) & 0xFF;
    buf[2]=(segment.ticks_cum >>24 ) & 0xFF;
    buf[9]=segment.millis_cum & 0xFF; //sizeof(segment.ticks_cum)); // 4 bytes
    buf[8]=(segment.millis_cum >>8 ) & 0xFF;
    buf[7]=(segment.millis_cum >>16 ) & 0xFF;
    buf[6]=(segment.millis_cum >>24 ) & 0xFF;
    buf[10]=segment.average_bearing; // 1 byte
    buf[11]=segment.current_bearing; // 1 byte
    buf[12]=(int((segment.speed_step+350)*255/MAX_SPEED/2)) & 0xFF; // converting [-350;+350] ticks/s to [0;255] - 127=0
//    Wire.write(rover.teta_point); // 2 bytes TBD
    if (seg_completed && (next_seg_available == false)){
      buf[13]=true; //1 byte
    }
    else
    {
      buf[13]=false; //1 byte
    }
//    while (i<14)
//    {
//      Serial.println(buf[i]);
//      i++;
//    }
    Wire.write(buf,14);
    I2Cregister=0;
  }
  if (I2Cregister==MOTORS_REGISTER){
    // 4 bytes to be sent    
    buf[0]=rover.FL_motor.power; //1 byte
    buf[1]=rover.FR_motor.power; //1 byte
    buf[2]=rover.RL_motor.power; //1 byte
    buf[3]=rover.RR_motor.power; //1 byte
    Wire.write(buf,4);
    I2Cregister=0;
  }
}

void driveRover(){
  if (segment.updateStatus()) { // Sensors could be read ok?
    // if new seg available, check segment Id to see if update or new and process accordingly, ie start new segment (n+1) or calcultae differential segment_target
    //  to be checked
    if ((next_seg_available) && (current_move.segment_id == next_move.segment_id)){ // MU sent an update of current segment
      current_move.target_ticks=next_move.target_ticks-segment.ticks_cum;
      if (current_move.target_ticks<0) {current_move.target_ticks=0;}
      current_move.target_bearing=next_move.target_bearing;
      current_move.target_speed=next_move.target_speed;
      next_seg_available=false;
      seg_completed=false;
      segment.begin();
    }
    if (seg_completed){ 
      if (next_seg_available) { // received segment and current one is completed - process new segment
        current_move.segment_type=next_move.segment_type; //0 in case rotation, 1 in case straight
        current_move.segment_id=next_move.segment_id;
        current_move.target_ticks=next_move.target_ticks;
        current_move.target_bearing=next_move.target_bearing;
        current_move.target_speed=next_move.target_speed;
        next_seg_available=false;
        seg_completed=false;
        segment.begin();
      }
      else { // seg completed but no new segmment is available => stop rover
        rover.FL_motor.stop();
        rover.FR_motor.stop();
        rover.RL_motor.stop();
        rover.RR_motor.stop();
      }
    } 
    else { // current SEG is not completed => continue current
      if (current_move.segment_type == 1){
        segment.deliverStraightSegment();
      }
      if (current_move.segment_type == 0){
        segment.deliverRotationSegment();
      }
      rover.move_rover();
    }   
  }
  else { // Sensors could not be read properly => stop rover as a safeguard measure
    rover.FL_motor.stop();
    rover.FR_motor.stop();
    rover.RL_motor.stop();
    rover.RR_motor.stop();
    Serial.print("Error reading sensors - rover stopped");      
  }
}

int i=0;

// Loop routine
void loop(){
   if (millis()-last_moment>200){ // for testing purpose
     last_moment=millis();
     test_motors(10);
     delay(200);

//     driveRover();
//     test_I2C_w_segment_receiving();
//     test_I2C_w_segment_receiving();
//      test_compass();
     test_decoders_read();
//     test_decoders();

   }
}


// ******************* Test routines - commented out in production version **************************** // 
void test_motors(int i){
  rover.FL_motor.myspeed=0; 
  rover.FR_motor.myspeed=0;        
  rover.RL_motor.myspeed=0;        
  rover.RR_motor.myspeed=0;
  rover.move_rover();
  Serial.println("motor running");
//  delay(1000);
//
//  rover.FL_motor.stop();
//  rover.FR_motor.stop();
//  rover.RL_motor.stop();
//  rover.RR_motor.stop();
////  rover.move_rover();
//  Serial.println("motor stopped");
//  delay(5000);
//
//
//  rover.FL_motor.myspeed=MAX_SPEED; 
//  rover.move_rover();
//  delay(1000);
//  rover.FL_motor.myspeed=0; 
//  rover.move_rover();
//
//  rover.FL_motor.myspeed=-MAX_SPEED; 
//  rover.move_rover();
//  delay(1000);
//  rover.FL_motor.myspeed=0; 
//  rover.move_rover();
//
//  delay(2000);
//
//  rover.FR_motor.myspeed=MAX_SPEED; 
//  rover.move_rover();
//  delay(1000);
//  rover.FR_motor.myspeed=0; 
//  rover.move_rover();
//
//  rover.FR_motor.myspeed=-MAX_SPEED; 
//  rover.move_rover();
//  delay(1000);
//  rover.FR_motor.myspeed=0; 
//  rover.move_rover();
//
//  delay(2000);
//
//  rover.RL_motor.myspeed=MAX_SPEED; 
//  rover.move_rover();
//  delay(1000);
//  rover.RL_motor.myspeed=0; 
//  rover.move_rover();
//
//  rover.RL_motor.myspeed=-MAX_SPEED; 
//  rover.move_rover();
//  delay(1000);
//  rover.RL_motor.myspeed=0; 
//  rover.move_rover();
//
//  delay(2000);
//
//  rover.RR_motor.myspeed=MAX_SPEED; 
//  rover.move_rover();
//  delay(1000);
//  rover.RR_motor.myspeed=0; 
//  rover.move_rover();
//
//  rover.RR_motor.myspeed=-MAX_SPEED; 
//  rover.move_rover();
//  delay(1000);
//  rover.RR_motor.myspeed=0;
//  rover.move_rover();
}

void test_decoders_basic(){ // test decoders and calibrate
  rover.FL_motor.myspeed=350;
  rover.FR_motor.myspeed=175;
  rover.RL_motor.myspeed=-175;
  rover.RR_motor.myspeed=-350;
  rover.move_rover();
}

void test_decoders_read(){ // test decoders and calibrate
//  rover.FL_motor.myspeed=100;
//  rover.FR_motor.myspeed=0;
//  rover.RL_motor.myspeed=0;
//  rover.RR_motor.myspeed=0;
//  rover.move_rover();
//  delay(50);
  segment.readDecoders();
  Serial.println(segment.FL_ticks_step);
  Serial.println(segment.FR_ticks_step);
  Serial.println(segment.RL_ticks_step);
  Serial.println(segment.RR_ticks_step);
  Serial.println(segment.millis_step);
}

void test_compass(){
  if (segment.readCompass()){
    Serial.print("Read successfull - ");
  }
  Serial.println(segment.current_bearing);
}

void test_segment_status(){ // test decoders and calibrate  byte segment_type; //0 in case rotation, 1 in case straight
  current_move.segment_type=1; //0 in case rotation, 1 in case straight
  current_move.segment_id=1;
  current_move.target_ticks=10000;
  current_move.target_bearing=90;
  current_move.target_speed=100; // Can vary between +350 and -350 converted from 1 byte SPI

  rover.FL_motor.myspeed=100;
  rover.FR_motor.myspeed=100;
  rover.RL_motor.myspeed=100;
  rover.RR_motor.myspeed=100;
  rover.move_rover();
  delay(50);
  segment.updateStatus();
  Serial.println("millis   - FL_cum   - RF_cum    - RL_cum    - RR_cum");
  Serial.print(segment.millis_cum);
  Serial.print(" ");
  Serial.print(segment.FL_ticks_cum);
  Serial.print(" ");
  Serial.print(segment.FR_ticks_cum);
  Serial.print(" ");
  Serial.print(segment.RL_ticks_cum);
  Serial.print(" ");
  Serial.println(segment.RR_ticks_cum);

  Serial.println("ticks step - ticks cum");
  Serial.print(segment.ticks_step);
  Serial.print(" ");
  Serial.println(segment.ticks_cum);
  
  Serial.println("speed step - speed cum");
  Serial.print(segment.speed_step);
  Serial.print(" ");
  Serial.println(segment.speed_cum);

  Serial.println("av bearing - gap cum - Completed");
  Serial.print(segment.average_bearing);
  Serial.print(" ");
  Serial.print(segment.gap_cum);
  Serial.print(" ");
  Serial.print(seg_completed);
}

void test_segment_delivery_straight(){ // test decoders and calibrate  byte segment_type; //0 in case rotation, 1 in case straight
  current_move.segment_type=1; //0 in case rotation, 1 in case straight
  current_move.segment_id=1;
  current_move.target_ticks=10000;
  current_move.target_bearing=90;
  current_move.target_speed=100; // Can vary between +350 and -350 converted from 1 byte SPI

  rover.FL_motor.myspeed=100;
  rover.FR_motor.myspeed=100;
  rover.RL_motor.myspeed=100;
  rover.RR_motor.myspeed=100;
  rover.move_rover();
  delay(50);

  segment.updateStatus();
  Serial.println("FL speed   - FR spee   - RL speed    - RR_cum    - RR_cum");
  Serial.print(segment.millis_cum);
  Serial.print(" ");

  rover.FL_motor.myspeed=100;
  rover.FR_motor.myspeed=100;
  rover.RL_motor.myspeed=100;
  rover.RR_motor.myspeed=100;

  Serial.println("millis   - FL_cum   - RF_cum    - RL_cum    - RR_cum");
  Serial.print(segment.millis_cum);
  Serial.print(" ");
  Serial.print(segment.FL_ticks_cum);
  Serial.print(" ");
  Serial.print(segment.FR_ticks_cum);
  Serial.print(" ");
  Serial.print(segment.RL_ticks_cum);
  Serial.print(" ");
  Serial.println(segment.RR_ticks_cum);

  Serial.println("ticks step - ticks cum");
  Serial.print(segment.ticks_step);
  Serial.print(" ");
  Serial.println(segment.ticks_cum);
  
  Serial.println("speed step - speed cum");
  Serial.print(segment.speed_step);
  Serial.print(" ");
  Serial.println(segment.speed_cum);

  Serial.println("av bearing - gap cum - Completed");
  Serial.print(segment.average_bearing);
  Serial.print(" ");
  Serial.print(segment.gap_cum);
  Serial.print(" ");
  Serial.print(seg_completed);
}

// Need a routine to test execution of a segment  rotation


void test_I2C_wo_segment_receiving(){
  current_move.segment_id=1; // 2 bytes
  segment.ticks_cum= 2; // 4 bytes
  segment.millis_cum= 3; // 4 bytes
  segment.average_bearing=4; // 1 byte
  segment.current_bearing=5; // 1 byte
  segment.speed_step=6; // 1 byte
  seg_completed=false;
  rover.FL_motor.power=128; //1 byte
  rover.FR_motor.power=129; //1 byte
  rover.RL_motor.power=130; //1 byte
  rover.RR_motor.power=131; //1 byte
}

void test_I2C_w_segment_receiving(){
  current_move.segment_id=1000; // 2 bytes
  segment.ticks_cum= 2000; // 4 bytes
  segment.millis_cum= 3000; // 4 bytes
  segment.average_bearing=100; // 1 byte
  segment.current_bearing=200; // 1 byte
  segment.speed_step=200; // 1 byte
  seg_completed=true;
  next_seg_available=false;
  rover.FL_motor.power=7; //1 byte
  rover.FR_motor.power=8; //1 byte
  rover.RL_motor.power=9; //1 byte
  rover.RR_motor.power=10; //1 byte
}

