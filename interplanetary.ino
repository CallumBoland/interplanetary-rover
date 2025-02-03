//drives the example robot
//uses MD25;  Servo;  Ultrasound distance sensor.

#include <Wire.h>  //library for I2C commuication with the MD25 board
#include <Math.h>  //maths

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

float encoder_to_mm = 0.9;  //NOT_USED multiply encoder value by this to get mm travelled. Calibrate based on wheel diameter.
float angle_to_dist = 2.5;  //Number of encoder steps to angular rotation (distance=angle*angle_to_dist on each wheel). Calibrate based on wheel diameter and axle length
float mm_to_dist = 1; //needs calibrating

//coordinate system(mm)
float bearing = 0;
long x = 0; //float?
long y = 0;
//constants
const long width = 200; //robot width and length
const long length = 300; //update as required
const long pivotDistance = 200; //distance from the front which the rover pivots about
const long sensorDistance = 150; //the distance between the two ultrasonic sensors
const long xMin = 0, xMax = 2400;
const long yMin = 0, yMax = 1600;
const int ultrasoundDelay = 100;
//location coordinates(mm)
const long targetCenter[] = {675,600};
const long rockSamples[] = {100,0};
const long cup1[] = {0,0}; //update as necessary
const long cup2[] = {0,1600};
//components
const int triggerPinR = 0, triggerPinL = 0, echoPinR = 0, echoPinL = 0;
const int inputpins[] = {triggerPinR, triggerPinL, echoPinR, echoPinL};
const int outputpins[] = {};
//prototypes (because arduino sucks)
void moveManhattan(long targetX, long targetY, int speed = 127, bool xFirst = true);
void aFindWall(int increments = 24, int speed = 127);
void aAlign(int iterations = 3, int speed = 127);
void aFindNorth(int speed = 127);


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
  
  delay(2500);     // This is just to give you a bit of time for you to clear your serial monitor before the robot gets going

  //assign pin modes
  for(int pin : inputpins){
    pinMode(pin,INPUT);
  }
  for(int pin : outputpins){
    pinMode(pin,OUTPUT);
  }


  //actual stuff
  taskA();
  delay(5000);
  taskB();
}

void loop() {
  //calibration 
  /*
  straight(200, 20);  //go forward 200 steps at speed 20  (note that forward/back and rotation direction will depend on which motor is plugged into which side
  turn(90, 10);      //turn 90 degrees clockwise at speed 10 
  delay(1500);
  */
}

void taskA(){
  //Task A: Navigation and self-location
  
  //rotate 360deg and find nearest wall
  aFindWall();
  //align with wall
  aAlign();
  //rotate in 90 degrees increments and find y-axis
  aFindNorth();
  //align with top edge and calibrate y coordinate
  aAlignY();//unfinished
  //move to ypos of target zone
  manhattanY(targetCenter[1],127);
  //slowly align with right wall and calibrate x coordinate
  aAlignX();//unfinished
  //move to xpos of target zone
  manhattanX(targetCenter[0],127);
  //done


}

void taskB(){
  //Task B: Sample collection and delivery 

  //go to samples and align
  //grab samples
  //deposit in cup1
  //deposit in cup2
  //celebrate
}

//task A stuff
void aFindWall(int increments, int speed){
  long closest = NULL;
  long distance;
  float nearest = 0;
  for(int i=0; i<=increments; i++){
    distance = measureR();
    if(distance<closest){
      closest = distance;
      nearest = (360.0/increments)*i;
    }
    turn(360.0/increments, speed);
  }
  turnToAngle(nearest, speed);
}

void aAlign(int iterations, int speed){
  for(int i=0; i<=3; i++){
    long distanceR = measureR();
    delay(ultrasoundDelay); //necessary(maybe)
    long distanceL = measureL();
    delay(ultrasoundDelay);
    float angle = atan((distanceL-distanceR)/sensorDistance)*(180/M_PI);
    turn(angle, speed);
  }
  bearing = 0;
  //potentially add section to distance self from nearby walls
}

void aFindNorth(int speed){
  long distancesR[4];
  long distancesL[4];
  long maximums[4];
  for(int i=0; i<=4; i++){
    distancesR[i] = measureR();
    delay(ultrasoundDelay);
    distancesL[i] = measureL();
    delay(ultrasoundDelay);
    turn(90,speed);
  }
  for(int i=0; i<=4; i++){
    maximums[i] = max(distancesR[i],distancesL[i]);
  }
  if(maximums[0]+maximums[2]>maximums[1]+maximums[3]){
    bearing = (maximums[0]<maximums[2])? 90 : -90;
  }
  else{
    bearing = (maximums[1]<maximums[3])? 0 : 180; 
  }
  turnToAngle(0,speed); //face north
}

void aAlignY(){

}

void aAlignX(){

}

//basic stuff
long measureR(){
  return(ultrasound(triggerPinR, echoPinR));
}

long measureL(){
  return(ultrasound(triggerPinL, echoPinL));
}

long ultrasound(int trigger, int echo){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  // Sets the trigger HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  // Reads the echoPin, returns the distance in millimetres
  return(pulseIn(echo, HIGH)*0.1715);
}

void moveManhattan(long targetX, long targetY, int speed, bool xFirst){
  if(xFirst){
    manhattanX(targetX,speed);
    manhattanY(targetY,speed);
  }
  else{
    manhattanY(targetY,speed);
    manhattanX(targetX,speed);
  }
}

void manhattanX(long targetX, int speed){
  if(targetX > x){
    turnToAngle(90, speed);
    straight(abs(targetX-x), speed);
  }
  else if(targetX < x){
    turnToAngle(-90, speed);
    straight(abs(targetX-x), speed);
  }
}
void manhattanY(long targetY, int speed){
  if(targetY > y){
    turnToAngle(0, speed);
    straight(abs(targetY-y), speed);
  }
  else if(targetY < y){
    turnToAngle(180, speed);
    straight(abs(targetY-y), speed);
  }
}

void moveDirect(long targetX, long targetY, int speed){ //will lose precision
  float dx = targetX-x, dy = targetY-y;
  float distance = sqrt(dx*dx+dy*dy);
  double angle = atan2(targetX,targetY)*(180/M_PI);//in degrees
  turnToAngle(angle, speed);
  straight(distance, speed);
}

void turnToAngle(float angle, int speed){
  float turnAngle = fmod(angle-bearing+180,360)-180;
  turn(turnAngle, speed);
}

//default stuff (mostly)
void turn(float angle, int speed)  //turn an angle in degrees at rate given by speed (values up to 127). negative angles OK.
{

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

void straight(float distance, int speed)  //go straight dist encoder steps at rate speed (values up to 127) -ve speed or dist goes backwards
{
  long encoderDistance = long(distance * mm_to_dist); //converts from mm to encoder steps
  float r = encoderDistance/mm_to_dist;
  x += r*cos(bearing*(M_PI/180)); //updates exact x and y coordinates
  y += r*sin(bearing*(M_PI/180));
  
  if (speed > 127) speed = 127;  //limit speed to maximum and minimum permitted value
  if (speed < -127) speed = -127;
  if (encoderDistance < 0) speed = -speed;  //this lets backwards motion be set by either negative distance or speed.

  motor1Speed = 128 + speed;  // int motor1Speed stores a value to be used as speed data for motor 1 (0 is full reverse, 128 is stop, 255 is full forward)
  motor2Speed = 128 + speed;  // int motor2Speed stores a value to be used as speed data for motor 2 (0 is full reverse, 128 is stop, 255 is full forward)
  motor1Dist = -encoderDistance;  // int motor1Dist stores the distance motor 1 will travel in the next motion
  motor2Dist = -encoderDistance;  // int motor2Dist stores the distance motor 2 will travel in the next motion

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

void set_speed1(char my_speed)    //set speed 1
{
   Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED1);
    Wire.write(my_speed);  // Sets the speed of motor 1 to a value of motor1Speed
    Wire.endTransmission();
}

void set_speed2(char my_speed)    //set speed 2
{
   Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED2);
    Wire.write(my_speed);  // Sets the speed of motor 1 to a value of motor1Speed
    Wire.endTransmission();
}

void moveMotor() {  // Function that moves the robot according to the values of motor1Speed, motor2Speed, motor1Dist and motor2Dist
  encoderReset();
  displayEncoderDist();   // Calls a function to read and display distance measured by encoders before movement begins (should be 0 or close to it)
  displayIntendedDist();  // Calls a function to display the distance the motors are intended to travel

  //in this while loop, both motors run until both have reached their target distance.  If one of them gets there first it is stoped while the other keeps going
  //only abs() values of distance are compared, so signs are not too important.  It's the speed that controls direction.
  while ((abs(encoder1()) < abs(motor1Dist)) || (abs(encoder2()) < abs(motor2Dist))) {  // while loop which continues to run if either encoder hasn't reached it's intended distance uses absolute values to convert vector distance into scalar. The brackets might look like overkill but it is to ensure correct order of operations

    displayEncoderDist();  // Calls a function to read and display distance measured by encoders during movement

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
  
 
  Serial.println(" ");
  displayEncoderDist();   // Calls a function to read and display distance measured by encoders after movement begins (should be close to intended travel distance)
  displayIntendedDist();  // Calls a function to display the distance the motors were intended to travel
}
