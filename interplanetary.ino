//drives the example robot
//uses MD25;  Servo;  Ultrasound distance sensor.

#include <Wire.h>   //library for I2C commuication with the MD25 board
#include <Math.h>   //maths
#include <EEPROM.h> //for saving callibration values
#include <Servo.h>  //for servos
#include <pitches.h> //for the buzzer

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
bool lastTaskState; //for switching tasks

//calibration values
float angle_to_dist = 3; //2.4558;  //Number of encoder steps per degree of rotation
float mm_to_dist = 1.146; //1.13; //Number of encoder steps per mm of travel (1/100xpi/360)
long pivotDistance = 160; //distance from the front which the rover pivots about
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
const int ultrasoundDelay = 50;
//location coordinates(mm)
const coordinate targetRough = {725,850};
const coordinate targetCenter = {725,605};
const coordinate rockSamples = {767,490};
const coordinate cup1 = {300,300}; //update as necessary
const coordinate cup2 = {300,1300};
//components
const int gate1Pin = 5, gate2Pin = 9, forkPin = 3; //servo pins
const int triggerPinR = 8, triggerPinL = 4, echoPinR = 7, echoPinL = 2; //ultrasound pins
const int taskSwitchPin = 13, leftFrontSwitch = 11, rightFrontSwitch = 12, buzzerPin = 10; 
const int inputpins[] = {echoPinR, echoPinL, taskSwitchPin, leftFrontSwitch, rightFrontSwitch}; //input pins
const int outputpins[] = {triggerPinR, triggerPinL, buzzerPin}; //output pins
//servos
Servo gate1;
Servo gate2;
Servo fork;
//buzzer
int melody[] = {NOTE_E5, NOTE_D5, NOTE_FS4, NOTE_GS4,  NOTE_CS5, NOTE_B4, NOTE_D4, NOTE_E4,  NOTE_B4, NOTE_A4, NOTE_CS4, NOTE_E4,  NOTE_A4};
int durations[] = {8, 8, 4, 4,  8, 8, 4, 4,  8, 8, 4, 4,  2};

//actual code
void setup() {
  Wire.begin();        // Begin I2C bus
  Serial.begin(9600);  // Begin serial
  delay(100);          // Wait for everything to power up
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
  digitalWrite(taskSwitchPin,HIGH);
  digitalWrite(rightFrontSwitch,HIGH);
  digitalWrite(leftFrontSwitch,HIGH);
  //attach servos
  gate1.attach(gate1Pin);
  gate2.attach(gate2Pin);
  fork.attach(forkPin);
  //setup servos
  bCloseGate(gate1);
  bOpenGateReverse(gate2);
  bStowFork();
  //delay(5000);
  //taskA();
  //delay(5000);
  //taskB();
  taskSelect(true);
}

void loop() {
  taskSelect(false);
}

void taskSelect(bool first){
  if(lastTaskState != digitalRead(taskSwitchPin) || first){
    if(digitalRead(taskSwitchPin)){
      delay(2000);
      taskA();
    }
    else {
      delay(2000);
      taskB();
    }
  }
  lastTaskState = digitalRead(taskSwitchPin);
}

void taskA(){
  //Task A: Navigation and self-location
  //rotate 360deg and find nearest wall
  //aFindWall(8, 20);
  //align with wall
  aAlign(0, 5, -1.0, 1, 0);
  //find position and bearing of rover
  aFindCoordinates(10);
  //move to target zone
  moveManhattan(targetRough.x,targetRough.y, 25, 10, false, false, false);
  //further alignment?
  aFinalAlign(10);
  //done
  buzz();
}

void taskB(){
  //Task B: Sample collection and delivery
  bearing = -90.0;
  x=targetCenter.x;
  y=targetCenter.y;
  //align x against left wall
  bAlignX(20);
  //move to xpos of samples
  manhattanX(rockSamples.x, 15, 10, true);
  turnToAngle(0, 10);
  delay(2000); //testing delay
  //grab samples
  bAlignY(20);
  bLowerFork();
  bGrabSamples(10);
  //reallign
  bCup1Realign(15);
  //deposit in cup1
  moveManhattan(cup1.x, cup1.y,15,10,false,false,true);
  turnToAngle(-135,10);
  bDeposit(10, 130, true);
  //reallign
  bCup2Realign(20,10);
  //deposit in cup2
  moveManhattan(cup2.x, cup2.y,15,10,true,true,false);
  turnToAngle(-45,10);
  bDeposit(10, 130, false);
  //celebrate
  buzz();
}

//task A stuff
void aFindWall(int increments, int speed){ //finds the rough angle of closest wall to the rover
  long r = measureR();
  long l = measureL();
  long closest = min(r,l);
  long distance;
  int nearest = 0;
  for(int i=0; i<increments; i++){ //rotate around 360deg
    r = measureR();
    l = measureL();
    distance = min(r,l);
    if(distance<closest){ //update angle as lower distances are found
      closest = distance;
      nearest = i;
    }
    turnToAngle((360.0/increments)*i, speed);//rotate in increments
  }
  turnToAngle(nearest*(360.0/increments), speed); //turn to face closest wall
  //add iterative rotation for accurate fast movement
}

void aAlign(int zero, int speed, float maxTurn, float exit, int layer){ //aligns the front face of the rover with the wall infront (multiple iterations may be required)
  bearing = zero;
  long distanceR = measureR();
  long distanceL = measureL();
  float angle = zero + atan((distanceL-distanceR)/sensorDistance)*(180/M_PI); //trig to find out angle that the rover needs to move
  if(abs(angle-zero)>maxTurn && maxTurn > 0 && layer<5){
    aAlign(zero,speed,maxTurn,exit, layer+1);
  }
  else{
    turnToAngle(angle, speed);
    bearing = zero;
    if(abs(angle-zero)>exit && exit > 0){
      aAlign(zero,speed,maxTurn,exit,layer);
    }
  }
}

void aFindCoordinates(int speed){ //calculates the bearing and rough position of the rover
  long distancesR[4]; //compares distances each cardinal direction
  long distancesL[4]; //finds the long edge of the arena, then finds the righthand edge
  long maximums[4];   //can then face the north edge using this information
  for(int i=0; i<4; i++){ //rotate around in 90deg increments
    distancesR[i] = measureR();
    distancesL[i] = measureL();
    if(i!=3){
    turn(90,speed);
    }
  }
  for(int i=0; i<4; i++){ //using maximum ranges to avoid detecting walls
    maximums[i] = max(distancesR[i],distancesL[i]);
    Serial.println(maximums[i]);
  }
  int rightIndex;
  if(maximums[0]+maximums[2]>maximums[1]+maximums[3]){ //find longest edge
    bearing = (maximums[0]<maximums[2])? 0 : 180; //find right hand wall and set bearing accordingly
    rightIndex = (maximums[0]<maximums[2])? 0 : 2;
  }
  else{
    bearing = (maximums[1]<maximums[3])? -90 : 90; 
    rightIndex = (maximums[1]<maximums[3])? 1 : 3;  
  }
  //calculate x coord from right hand wall
  x = xMax - (maximums[rightIndex] + pivotDistance);
  //calculate y coord from closest y wall
  y = maximums[(rightIndex+3)%4]<maximums[(rightIndex+1)%4]? yMax - (maximums[(rightIndex+3)%4] + pivotDistance):(maximums[(rightIndex+1)%4] + pivotDistance);
}

void aFinalAlign(int speed){
  aAlign(-90, 5, -1.0, 3.0, 0);
  x=((measureR()+measureL())/2) + pivotDistance;
  turnToAngle(0,speed);
  y=yMax-(((measureR()+measureL())/2) + pivotDistance);
  //y align
  straight(targetCenter.y-y,speed);
  //x alignment
  turnToAngle(-90,speed);
  straight(x-targetCenter.x,speed);
}

//task B stuff
void bAlignX(int speed){
  turnToAngle(-90,speed); //face west
  alignmentMove(speed); //ram left wall
  x=pivotDistance; //set zero
  bearing = -90;//reset bearing to align
}

void bAlignY(int speed){
  turnToAngle(0,speed); //face north
  y=yMax-(measureR()+measureL())/2-pivotDistance;
  straight(rockSamples.y+150-y,speed);
}

void bGrabSamples(int speed){
  //grab first two
  straight(rockSamples.y-y,speed);
  bLiftFork();
  delay(1000); //wait for balls to move
  straight(20,speed);
  bLowerFork();
  bCloseGate(gate2);
  //grab second two
  straight(-125,speed); //reverse
  bLiftFork();
  delay(1000); //wait for balls to move
  bStowFork();
  //move out
  straight(100,speed);
}

void bDeposit(int speed, long forward, bool firstDrop){
  straight(forward, speed);
  bOpenGate(gate1);
  delay(2000);
  if(firstDrop){
    bCloseGate(gate1);
    delay(100);
    bOpenGate(gate2);
  }
  straight(-forward, speed);
}

void bCup1Realign(int speed){
  bAlignX(speed);
  straight(-300,speed);
  turnToAngle(180,speed);
  alignmentMove(speed);
  y=pivotDistance;
}

void bCup2Realign(int speed, int preciseSpeed){
  bAlignX(speed);
  straight(-250,speed);
  turnToAngle(0,speed);
  alignmentMove(speed);
  y=yMax-pivotDistance;
  straight(-250,preciseSpeed);
  bAlignX(preciseSpeed);
}

//buzzer stuff
void buzz(){
  int size = sizeof(durations) / sizeof(int);
  for (int note = 0; note < size; note++) {
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 1000 / durations[note];
    tone(buzzerPin, melody[note], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    
    //stop the tone playing:
    noTone(buzzerPin);
  }
}

//servo control
void slowServo(Servo servo, int start, int end, int d){
  if (start<end) {
    for(int i = start/10; i<end/10; i++){
      servo.writeMicroseconds(i*10);
      delay(d);
    }
  }
  else{
    for(int i = start/10; i>end/10; i--){
      servo.writeMicroseconds(i*10);
      delay(d);
    }
  }
  servo.writeMicroseconds(end);
}

void bLiftFork(){
  slowServo(fork, fork.readMicroseconds(), 1200, 20);
}

void bLowerFork(){
  slowServo(fork, fork.readMicroseconds(), 625, 10);
}

void bStowFork(){
  slowServo(fork, fork.readMicroseconds(), 1950, 10);
}

void bOpenGate(Servo gate){
  gate.write(0);
}

void bCloseGate(Servo gate){
  gate.write(90);
}

void bOpenGateReverse(Servo gate){
  gate.write(180);
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

void moveManhattan(long targetX, long targetY, int speed, int turnSpeed, bool xFirst, bool reverseX, bool reverseY){ //moves to a point along the x and y axis
  if(xFirst){ //move via x-axis first
    manhattanX(targetX,speed,turnSpeed,reverseX);
    manhattanY(targetY,speed,turnSpeed,reverseY);
  }
  else{ //move via y-axis first
    manhattanY(targetY,speed,turnSpeed,reverseY);
    manhattanX(targetX,speed,turnSpeed,reverseX);
  }
}

void manhattanX(long targetX, int speed, int turnSpeed, bool reverse){ //move along the x-axis to a given point
  int reverseMult = reverse? -1: 1;
  if((targetX > x && !reverse)||(targetX < x && reverse)){ //face the correct direction and then move forward
    turnToAngle(90, turnSpeed);
    straight(abs(targetX-x)*reverseMult, speed);
  }
  else if((targetX < x && !reverse)||(targetX > x && reverse)){
    turnToAngle(-90, turnSpeed);
    straight(abs(targetX-x)*reverseMult, speed);
  }
}
void manhattanY(long targetY, int speed, int turnSpeed, bool reverse){ //move along the y-axis to a given point
  int reverseMult = reverse? -1: 1;
  if((targetY > y && !reverse)||(targetY < y && reverse)){ //face the correct direction and then move forward
    turnToAngle(0, turnSpeed);
    straight(abs(targetY-y)*reverseMult, speed);
  }
  else if((targetY < y && !reverse)||(targetY > y && reverse)){
    turnToAngle(180, turnSpeed);
    straight(abs(targetY-y)*reverseMult, speed);
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
  delay(100);
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
  moveMotor(true);
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

  moveMotor(true);
}

void encoderReset() {  // This function resets the encoder values to 0
  delay(150);          // This delay is important (especially when using odometry), it gives the robot some time to come to rest before resetting the encoder, reducing the chance of momentum affecting your encoder values
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(RESETENCODERS);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(25);
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

void correctMovement(int maxSpeed){ //corrects overshoot of the motors
  motor1Dist = abs(encoder1())-abs(motor1Dist); //sets 
  motor1Speed = 128+constrain(128-motor1Speed,-maxSpeed,maxSpeed);
  motor2Dist = abs(encoder2())-abs(motor2Dist);
  motor2Speed = 128+constrain(128-motor2Speed,-maxSpeed,maxSpeed);
  moveMotor(false);
}

void moveMotor(bool correction) {  // Function that moves the robot according to the values of motor1Speed, motor2Speed, motor1Dist and motor2Dist
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

  //corrections
  if(correction){
    correctMovement(3);
  }
  
  /*
  Serial.println(" ");
  displayEncoderDist();   // Calls a function to read and display distance measured by encoders after movement begins (should be close to intended travel distance)
  displayIntendedDist();  // Calls a function to display the distance the motors were intended to travel
  */
}
