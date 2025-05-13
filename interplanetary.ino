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
  motor1Dist = abs(encoder1())-abs(motor1Dist); //sets overshoot distance
  motor1Speed = 128+constrain(128-motor1Speed,-maxSpeed,maxSpeed); //sets direction and speed
  motor2Dist = abs(encoder2())-abs(motor2Dist); 
  motor2Speed = 128+constrain(128-motor2Speed,-maxSpeed,maxSpeed);
  moveMotor(false); //moves without correcting again
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
  if(correction && caution){
    correctMovement(3);
  }
  
  /*
  Serial.println(" ");
  displayEncoderDist();   // Calls a function to read and display distance measured by encoders after movement begins (should be close to intended travel distance)
  displayIntendedDist();  // Calls a function to display the distance the motors were intended to travel
  */
}
