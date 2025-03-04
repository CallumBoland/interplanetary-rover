//drives the example robot
//uses MD25;  Servo;  Ultrasound distance sensor.

#include <Wire.h>   //library for I2C commuication with the MD25 board
#include <Math.h>   //maths
#include <EEPROM.h> //for saving callibration values
#include <Servo.h>  //for servos

//define lots of constants telling arduino about the MD25 driver board and how to access its registers
#define CMD (byte)0x00      // Values of 0 being sent using write have to be cast as a byte to stop them being misinterperted as NULL (This is a bug with arduino 1)
#define MD25ADDRESS 0x58    // Address of the MD25
#define SPEED1 (byte)0x00   // Byte to send speed to first motor
#define SPEED2 0x01         // Byte to send speed to second motor
#define ENCODERONE 0x02     // Byte to read motor encoder 1
#define ENCODERTWO 0x06     // Byte to read motor encoder 2
#define RESETENCODERS 0x10  // Byte to reset encoder values
#define MODESELECTOR 0xF    // Byte to change between control MODES

//these are global variables
int mode = 0;           // int mode stores the mode in which the MD25 will operate
int motor1Speed = 128;  // int motor1Speed stores a value to be used as speed data for motor 1 (0 is full reverse, 128 is stop, 255 is full forward)
int motor2Speed = 128;  // int motor2Speed stores a value to be used as speed data for motor 2 (0 is full reverse, 128 is stop, 255 is full forward)
long motor1Dist = 0;     // int motor1Dist stores a value to be used as the distance in encoder units motor 1 will travel in a single motion
long motor2Dist = 0;     // int motor2Dist stores a value to be used as the distance in encoder units motor 2 will travel in a single motion

//calibration values
float angle_to_dist = 3.1; //2.4558;  //Number of encoder steps per degree of rotation
float mm_to_dist = 1.146; //1.13; //Number of encoder steps per mm of travel (1/100xpi/360)
long pivotDistance = 155; //distance from the front which the rover pivots about
//calibration EEPROM struct
struct EEPROMvalues{
  float angle;
  float mm;
  long pivot;
};
//coordinate struct
struct coordinate{
  long x;
  long y;
};
//coordinate system(mm)
float bearing = 0;
long x = 0; //float?
long y = 0;
//constants
const long width = 200; //robot width and length
const long length = 300; //update as required
const float sensorDistance = 148; //the distance between the two ultrasonic sensors
const long xMin = 0, xMax = 2400; //arena size
const long yMin = 0, yMax = 1600;
const int ultrasoundDelay = 10;
//location coordinates(mm)
const coordinate targetCenter = {675,600};
const coordinate rockSamples = {100,0};
const coordinate cup1 = {0,0}; //update as necessary
const coordinate cup2 = {0,1600};
//components
const int gate1Pin = 11, gate2Pin = 10, forkPin = 9; //servo pins
const int triggerPinR = 8, triggerPinL = 4, echoPinR = 7, echoPinL = 2; //ultrasound pins
const int calibrationButtonPin = 3, leftFrontSwitch = 12, rightFrontSwitch = 13; 
const int inputpins[] = {echoPinR, echoPinL, calibrationButtonPin, leftFrontSwitch, rightFrontSwitch}; //input pins
const int outputpins[] = {triggerPinR, triggerPinL}; //output pins
//servos
Servo gate1;
Servo gate2;
Servo fork;
//prototypes (because arduino compliler sucks)
void moveManhattan(long targetX, long targetY, int speed = 20, bool xFirst = true);
void aFindWall(int increments = 12, int speed = 10);
float aAlign(int speed = 20);
void aFindNorth(int speed = 20);

//actual code
void setup() {
  Wire.begin();        // Begin I2C bus
  Serial.begin(9600);  // Begin serial
  delay(100);          // Wait for everything to power up
  Serial.println("jhole");
  Wire.beginTransmission(MD25ADDRESS);  // Set MD25 operation to MODE given by mode
  Wire.write(MODESELECTOR);
  Wire.write(mode);
  Wire.endTransmission();

  encoderReset();  // Calls a function which resets the encoder values to 0

  //assign pin modes
  for(int pin : inputpins){
    pinMode(pin,INPUT);
  }
  for(int pin : outputpins){
    pinMode(pin,OUTPUT);
  }
  digitalWrite(rightFrontSwitch,HIGH);
  digitalWrite(leftFrontSwitch,HIGH);
  //attach servos
  gate1.attach(gate1Pin);
  gate2.attach(gate2Pin);
  fork.attach(forkPin);
  //setup servos
  bCloseGate(gate1);
  bOpenGate(gate2);
  bStowFork();

  delay(5000);
  /*
  if(digitalRead(calibrationButtonPin)){
    calibrate(5);
  }
  else {
  */
  taskA();
  //delay(5000);
  //taskB();
}

void loop() {

}

void readEEPROM(){
  EEPROMvalues read;
  EEPROM.get(0, read);
  angle_to_dist = read.angle;
  mm_to_dist = read.mm;
  pivotDistance = read.pivot;
}
void writeEEPROM(float angle, float mm, long pivot){
  EEPROMvalues write = {angle,mm,pivot};
  EEPROM.put(0, write);
}

void taskA(){
  //Task A: Navigation and self-location
  //rotate 360deg and find nearest wall
  aFindWall(8, 10);
  //align with wall
  aAlign(5);
  aAlign(5);
  aAlign(5);
  //find position and bearing of rover
  aFindNorth(10);
  //move to target zone
  moveManhattan(targetCenter.x,targetCenter.y,20,false);
  //further alignment?
  //done
}

void taskB(){
  //Task B: Sample collection and delivery
  //align x against left wall
  //bAlignX(10);
  //move to xpos of samples
  //manhattanX();
  turnToAngle(0,10);
  //grab samples
  bLowerFork();
  bGrabSamples(10);
  //deposit in cup1
  moveManhattan(cup1.x, cup1.y,10);
  turnToAngle(-135,10);
  bDeposit(true);
  //deposit in cup2
  moveManhattan(cup2.x, cup2.y,10);
  turnToAngle(45,10);
  bDeposit(false);
  //celebrate
}

void calibrate(int speed){
  //angle
  Serial.println(angle_to_dist);//outputs just incase
  Serial.println(mm_to_dist);
  Serial.println(pivotDistance);
  angle_to_dist = 1; //reset for calibration
  mm_to_dist = 1;
  pivotDistance = 0;
  delay(10000);
  angle_to_dist = cAngle(speed); //place in top-right corner facing north
  //distance
  delay(5000);
  mm_to_dist = cDistance(speed); //place back against right wall
  //pivot
  delay(5000);
  pivotDistance = cPivot(speed); //place back against any wall
  //done?
  writeEEPROM(angle_to_dist, mm_to_dist, pivotDistance);
}
//calibration stuff
float cAngle(int speed){
  aAlign(); //align (may suck actually)
  long steps = 10;
  long distanceR = 0;
  long distanceL = 0;
  turn(10,speed);
  while(!(distanceL>=distanceR)){ //for rotation <45deg
    turn(1,speed);
    steps+=1;
    distanceR = measureR();
    distanceL = measureL();
  }
  turn(10,speed);
  steps+=10;
  while(!(distanceR>=distanceL)){ //up to 90deg
    turn(1,speed);
    steps+=1;
    distanceR = measureR();
    distanceL = measureL();
  }
  float tempAngle = steps/(90.0+atan((distanceL-distanceR)/sensorDistance)*(180/M_PI));//get rough(good) estimate for constant
  turn(long(1800*tempAngle),speed); //5 full turns for precision
  steps+=long(1800*tempAngle);
  distanceR = measureR();
  distanceL = measureL();
  tempAngle = steps/(1800.0+90.0+atan((distanceL-distanceR)/sensorDistance)*(180/M_PI));//best estimate for constant
  return tempAngle;
}

float cDistance(int speed){
  straight(-10.0,speed);//move backwards for flush alignment
  long steps=0;
  while(!digitalRead(rightFrontSwitch)){ //trudge on until wall met
    straight(1,speed);
    steps+=1;
  }
  return float(steps)/(xMax-length); //return constant
}

long cPivot(int speed){
  straight(-10.0,speed);
  straight(400,speed);
  turn(180,speed);
  long distanceR = measureR();
  long distanceL = measureL();
  return (400+length-(distanceR+distanceL)/2)/2; //no way this is accurate
}

//task A stuff
void aFindWall(int increments, int speed){ //finds the rough angle of closest wall to the rover
  long closest = measureR()+measureL();
  delay(ultrasoundDelay);
  long distance;
  float nearest = 0;
  for(int i=0; i<=increments; i++){ //rotate around 360deg
    distance = measureR()+measureL();
    if(distance<closest){ //update angle as lower distances are found
      closest = distance;
      nearest = (360.0/increments)*i;
    }
    turnToAngle((360.0/increments)*i, speed);//rotate in increments
  }
  turnToAngle(nearest, speed); //turn to face closest wall
  //add iterative rotation for accurate fast movement
}

float aAlign(int speed){ //aligns the front face of the rover with the wall infront (multiple iterations may be required)
  bearing = 0;
  long distanceR = measureR();
  long distanceL = measureL();
  float angle = atan((distanceL-distanceR)/sensorDistance)*(180/M_PI); //trig to find out angle that the rover needs to move
  turnToAngle(angle, speed);
  bearing = 0;
  return angle; // returns so it may be iterated outside the function
}

void aFindNorth(int speed){ //locates the 'north' direction
  long distancesR[4]; //compares distances each cardinal direction
  long distancesL[4]; //finds the long edge of the arena, then finds the righthand edge
  long maximums[4];   //can then face the north edge using this information
  for(int i=0; i<4; i++){ //rotate around in 90deg increments
    distancesR[i] = measureR();
    distancesL[i] = measureL();
    turn(90,speed); //add if statement?
  }
  for(int i=0; i<4; i++){ //using maximum ranges to avoid detecting walls
    maximums[i] = max(distancesR[i],distancesL[i]);
    Serial.println(maximums[i]);
  }
  int rightIndex;
  if(maximums[0]+maximums[2]>maximums[1]+maximums[3]){ //find longest edge
    bearing = (maximums[0]<maximums[2])? 90 : -90; //find right hand wall and set bearing accordingly
    rightIndex = (maximums[0]<maximums[2])? 0 : 2;
  }
  else{
    bearing = (maximums[1]<maximums[3])? 0 : 180; 
    rightIndex = (maximums[1]<maximums[3])? 1 : 3;  
  }
  //calculate x coord from right hand wall
  x = xMax - (maximums[rightIndex] + pivotDistance);
  //calculate y coord from closest y wall
  y = maximums[(rightIndex+3)%4]<maximums[(rightIndex+1)%4]? yMax - (maximums[(rightIndex+3)%4] + pivotDistance):(maximums[(rightIndex+1)%4] + pivotDistance);
}

void aAlignY(int speed){
  //no need to turn, already facing north
  alignmentMove(speed); //ram into wall
  y=yMax-pivotDistance; //set ypos
  bearing = 0; //reset bearing to align
  straight(-100,speed); //move away from wall to allow turning
}

void aAlignX(int speed){
  turnToAngle(90,speed); //face east
  alignmentMove(speed); //ram right wall
  x=xMax-pivotDistance; //set zero
  bearing = 90;//reset bearing to align
  straight(-100,speed); //move away from wall to allow turning
}
//task B stuff
void bAlign(int speed){
  manhattanX(rockSamples.x,speed);
  //face north
  //reverse onto
}

void bGrabSamples(int speed){
  //grab first two
  straight(-100,speed); //reverse
  bLiftFork();
  delay(1000); //wait for balls to move
  bLowerFork();
  bCloseGate(gate2);
  //grab second two
  straight(-100,speed); //reverse
  bLiftFork();
  delay(1000); //wait for balls to move
  bStowFork();
  //move away
  straight(200,speed); //move forward
}

void bDeposit(bool firstDrop){
  bOpenGate(gate1);
  if(!firstDrop){
  bOpenGate(gate2);
  }
}


//servo control
void bLiftFork(){
  fork.write(90);
}

void bLowerFork(){
  fork.write(0);
}

void bStowFork(){
  fork.write(180);
}

void bOpenGate(Servo gate){
  gate.write(180);
}

void bCloseGate(Servo gate){
  gate.write(0);
}

//basic stuff
long measureR(){
  delay(ultrasoundDelay); //necessary delay
  return(ultrasound(triggerPinR, echoPinR));
}

long measureL(){
  delay(ultrasoundDelay); //necessary delay
  return(ultrasound(triggerPinL, echoPinL));
}

long ultrasound(int trigger, int echo){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  // Sets the trigger HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  // Reads the echoPin, returns the distance in mm
  return(pulseIn(echo, HIGH)*0.1715);
}

void moveManhattan(long targetX, long targetY, int speed, bool xFirst){ //moves to a point along the x and y axis
  if(xFirst){ //move via x-axis first
    manhattanX(targetX,speed);
    manhattanY(targetY,speed);
  }
  else{ //move via y-axis first
    manhattanY(targetY,speed);
    manhattanX(targetX,speed);
  }
}

void manhattanX(long targetX, int speed){ //move along the x-axis to a given point
  if(targetX > x){ //face the correct direction and then move forward
    turnToAngle(90, speed);
    straight(abs(targetX-x), speed);
  }
  else if(targetX < x){
    turnToAngle(-90, speed);
    straight(abs(targetX-x), speed);
  }
}
void manhattanY(long targetY, int speed){ //move along the y-axis to a given point
  if(targetY > y){ //face the correct direction and then move forward
    turnToAngle(0, speed);
    straight(abs(targetY-y), speed);
  }
  else if(targetY < y){
    turnToAngle(180, speed);
    straight(abs(targetY-y), speed);
  }
}

void moveDirect(long targetX, long targetY, int speed){ //will lose precision
  float dx = targetX-x, dy = targetY-y; //find change in distance
  float distance = sqrt(dx*dx+dy*dy); //find direct distance
  double angle = atan2(dx,dy)*(180/M_PI);//find bearing to travel in degrees
  turnToAngle(angle, speed); //turn
  straight(distance, speed); //move
}

void turnToAngle(float angle, int speed){
  float turnAngle = fmod(angle-bearing+180,360)-180; //normalise angle
  turn(turnAngle, speed);
}

void alignmentMove(int speed){
  while(!(digitalRead(leftFrontSwitch)&&digitalRead(rightFrontSwitch))){ //while not in contact with a wall
    set_speed1(128-speed); //move forwards at speed
    set_speed2(128-speed);
  }
  set_speed1(128); //halt
  set_speed2(128);
}
//default stuff (mostly)
void turn(float angle, int speed){  //turn an angle in degrees at rate given by speed (values up to 127). negative angles OK.

  long ang_dist = long(angle_to_dist * angle);  //get number of encoder steps required to give angle

  bearing += ang_dist/angle_to_dist; //updates bearing according to encoder steps
  bearing = fmod(bearing+180,360)-180; // limits to +-180 degrees

  if (speed > 127) speed = 127;  //limit speed to maximum and minimum permitted value
  if (speed < -127) speed = -127;
  if (angle < 0) speed = -speed;  //reverse direction for negative angle

  motor1Speed = 128 + speed;   // int motor1Speed stores a value to be used as speed data for motor 1 (0 is full reverse, 128 is stop, 255 is full forward)
  motor2Speed = 128 - speed;   // int motor2Speed stores a value to be used as speed data for motor 2 (0 is full reverse, 128 is stop, 255 is full forward)
  motor1Dist = abs(ang_dist);  // int motor1Dist stores the distance motor 1 will travel in the next motion
  motor2Dist = abs(ang_dist);  // int motor2Dist stores the distance motor 2 will travel in the next motion
  moveMotor();
}

void straight(float distance, int speed){  //go straight dist encoder steps at rate speed (values up to 127) -ve speed or dist goes backwards
  long encoderDistance = -long(distance * mm_to_dist); //converts from mm to encoder steps
  float r = -encoderDistance/mm_to_dist;
  r = speed>0 ? r:-r; //account for negative speed
  x += r*sin(bearing*(M_PI/180)); //updates exact x and y coordinates
  y += r*cos(bearing*(M_PI/180));
  
  if (speed > 127) speed = 127;  //limit speed to maximum and minimum permitted value
  if (speed < -127) speed = -127;
  if (encoderDistance < 0) speed = -speed;  //this lets backwards motion be set by either negative distance or speed.

  motor1Speed = 128 + speed;  // int motor1Speed stores a value to be used as speed data for motor 1 (0 is full reverse, 128 is stop, 255 is full forward)
  motor2Speed = 128 + speed;  // int motor2Speed stores a value to be used as speed data for motor 2 (0 is full reverse, 128 is stop, 255 is full forward)
  motor1Dist = encoderDistance;  // int motor1Dist stores the distance motor 1 will travel in the next motion
  motor2Dist = encoderDistance;  // int motor2Dist stores the distance motor 2 will travel in the next motion

  moveMotor();
}

void encoderReset() {  // This function resets the encoder values to 0
  delay(250);          // This delay is important (especially when using odometry), it gives the robot some time to come to rest before resetting the encoder, reducing the chance of momentum affecting your encoder values
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(RESETENCODERS);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(50);
}

long encoder1() {                       // Function to read the value of encoder 1 as a long (32 bit
  Wire.beginTransmission(MD25ADDRESS);  // Send byte to get a reading from encoder 1
  Wire.write(ENCODERONE);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);  // Request 4 bytes from MD25
  while (Wire.available() < 4)
    ;                        // Wait for 4 bytes to become available
  long poss1 = Wire.read();  // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read();  // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();  // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1 += Wire.read();  // Fourth byte for encoder 1, LL
  delay(5);              // Wait to make sure everything is sent
  return (poss1);        // return encoder value
}

long encoder2() {                       // Function to read the value of encoder 2 as a long (32bit signed)
  Wire.beginTransmission(MD25ADDRESS);  // Send byte to get a reading from encoder 2
  Wire.write(ENCODERTWO);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4);  // Request 4 bytes from MD25
  while (Wire.available() < 4)
    ;                        // Wait for 4 bytes to become available
  long poss2 = Wire.read();  // First byte for encoder 2, HH
  poss2 <<= 8;
  poss2 += Wire.read();  // Second byte for encoder 2, HL
  poss2 <<= 8;
  poss2 += Wire.read();  // Third byte for encoder 2, LH
  poss2 <<= 8;
  poss2 += Wire.read();  // Fourth byte for encoder 2, LL
  delay(5);              // Wait to make sure everything is sent
  return (poss2);        // return encoder value
}

void displayEncoderDist() {  // Function to read and display distance measured by encoders
  Serial.print("Encoder 1 Distance mm - ");
  Serial.print(encoder1());  // Reads and displays last recorded traveled distance of motor 1
  Serial.print("   ");
  Serial.print("Encoder 2 Distance mm - ");
  Serial.print(encoder2());  // Reads and displays last recorded traveled distance of motor 2
  Serial.println(" ");
}

void displayIntendedDist() {  // Function to display the distance the motors are inteded to travel
  Serial.print("motor1Dist mm - ");
  Serial.print(motor1Dist);  // Displays intended travel distance of motor 1
  Serial.print("   ");
  Serial.print("motor2Dist mm - ");
  Serial.print(motor2Dist);  // Displays intended travel distance of motor 2
  Serial.println(" ");
  Serial.println(" ");
}

void set_speed1(char my_speed){    //set speed 1
   Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED1);
    Wire.write(my_speed);  // Sets the speed of motor 1 to a value of motor1Speed
    Wire.endTransmission();
}

void set_speed2(char my_speed){    //set speed 2
   Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED2);
    Wire.write(my_speed);  // Sets the speed of motor 1 to a value of motor1Speed
    Wire.endTransmission();
}

void moveMotor() {  // Function that moves the robot according to the values of motor1Speed, motor2Speed, motor1Dist and motor2Dist
  encoderReset();
  //displayEncoderDist();   // Calls a function to read and display distance measured by encoders before movement begins (should be 0 or close to it)
  //displayIntendedDist();  // Calls a function to display the distance the motors are intended to travel

  //in this while loop, both motors run until both have reached their target distance.  If one of them gets there first it is stoped while the other keeps going
  //only abs() values of distance are compared, so signs are not too important.  It's the speed that controls direction.
  while ((abs(encoder1()) < abs(motor1Dist)) || (abs(encoder2()) < abs(motor2Dist))) {  // while loop which continues to run if either encoder hasn't reached it's intended distance uses absolute values to convert vector distance into scalar. The brackets might look like overkill but it is to ensure correct order of operations

    //displayEncoderDist();  // Calls a function to read and display distance measured by encoders during movement

    set_speed1(motor1Speed);  // Sets the speed of motor 1 to a value of motor1Speed
    set_speed2(motor2Speed);  // Sets the speed of motor 2 to a value of motor2Speed
    
   
    if ((motor1Speed != 128) && (abs(encoder1()) >= abs(motor1Dist))) {  // if statement triggers if motor 1 is still moving after reaching it's intended travel distance. The brackets might look like overkill but it is to ensure correct order of operations
      set_speed1(128);                                                 // STOP MOTOR 1  (128 is stop)
    }
    if ((motor2Speed != 128) && (abs(encoder2()) >= abs(motor2Dist))) {  // if statement triggers if motor 2 is still moving after reaching it's intended travel distance. The brackets might look like overkill but it is to ensure correct order of operations
      set_speed2(128);                                                 // STOP MOTOR 2  (128 is stop)  
    }
  }
  // The next 2 wire transmissions might seem unnecesary but they are essential as the encoder values are updated each time encoder1() and encoder2() are called meaning the while loop can exit without triggering the if statements. There are a couple of other ways around this problem but I found this the most elegant
  set_speed1(128); // Sets the speed of motor 1 to 128 (stop)
  set_speed2(128); // Sets the speed of motor 2 to 128 (stop)
  
  /*
  Serial.println(" ");
  displayEncoderDist();   // Calls a function to read and display distance measured by encoders after movement begins (should be close to intended travel distance)
  displayIntendedDist();  // Calls a function to display the distance the motors were intended to travel
  */
}
