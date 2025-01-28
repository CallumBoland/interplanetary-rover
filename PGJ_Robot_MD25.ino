//drives the example robot
//uses MD25;  Servo;  Ultrasound distance sensor.


#include <Wire.h>  //library for I2C commuication with the MD25 board

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
int motor1Dist = 0;     // int motor1Dist stores a value to be used as the distance in encoder units motor 1 will travel in a single motion
int motor2Dist = 0;     // int motor2Dist stores a value to be used as the distance in encoder units motor 2 will travel in a single motion

float encoder_to_mm = 0.9;  //NOT_USED multiply encoder value by this to get mm travelled. Calibrate based on wheel diameter.
float angle_to_dist = 2.5;  //Number of encoder steps to angular rotation (distance=angle*angle_to_dist on each wheel). Calibrate based on wheel diameter and axle length


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
}


void loop() {

  my_straight(200, 20);  //go forward 200 steps at speed 20  (note that forward/back and rotation direction will depend on which motor is plugged into which side
  my_turn(90, 10);      //turn 90 degrees clockwise at speed 10 
  delay(1500);
  //This keeps repeating and robot will move in a square shaper (though calibration may be needed to get andgles accurate)
  //Note that if you go faster, then robot momentum adds more to innaccuracy.
  //Experiment with measuring the repeatability of your robot movement (eg returning to known start point), and consider
  //ways to improve this - e.g. measure actual distance travelled with encoder or use acceleration and deceleration to allow faster movemenet without momentum effects.
  //In this code all distances are in encoder units, but you will want to work in mm in most of your code.

 
}


void my_turn(int angle, int speed)  //turn an angle in degrees at rate given by speed (values up to 127). negative angles OK.
{

  int ang_dist = angle_to_dist * angle;  //get number of encoder steps required to give angle

  if (speed > 127) speed = 127;  //limit speed to maximum and minimum permitted value
  if (speed < -127) speed = -127;
  if (angle < 0) speed = -speed;  //reverse direction for negative angle

  motor1Speed = 128 + speed;   // int motor1Speed stores a value to be used as speed data for motor 1 (0 is full reverse, 128 is stop, 255 is full forward)
  motor2Speed = 128 - speed;   // int motor2Speed stores a value to be used as speed data for motor 2 (0 is full reverse, 128 is stop, 255 is full forward)
  motor1Dist = abs(ang_dist);  // int motor1Dist stores the distance motor 1 will travel in the next motion
  motor2Dist = abs(ang_dist);  // int motor2Dist stores the distance motor 2 will travel in the next motion
  moveMotor();
}

void my_straight(int dist, int speed)  //go straight dist encoder steps at rate speed (values up to 127) -ve speed or dist goes backwards
{
  dist=-dist;    //motors are backwards, so this makes +ve distance move forwards.
  if (speed > 127) speed = 127;  //limit speed to maximum and minimum permitted value
  if (speed < -127) speed = -127;
  if (dist < 0) speed = -speed;  //this lets backwards motion be set by either negative distance or speed.

  motor1Speed = 128 + speed;  // int motor1Speed stores a value to be used as speed data for motor 1 (0 is full reverse, 128 is stop, 255 is full forward)
  motor2Speed = 128 + speed;  // int motor2Speed stores a value to be used as speed data for motor 2 (0 is full reverse, 128 is stop, 255 is full forward)
  motor1Dist = dist;          // int motor1Dist stores the distance motor 1 will travel in the next motion
  motor2Dist = dist;          // int motor2Dist stores the distance motor 2 will travel in the next motion

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
    
   
    if (((motor1Speed != 128) && abs(encoder1()) >= abs(motor1Dist))) {  // if statement triggers if motor 1 is still moving after reaching it's intended travel distance. The brackets might look like overkill but it is to ensure correct order of operations
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