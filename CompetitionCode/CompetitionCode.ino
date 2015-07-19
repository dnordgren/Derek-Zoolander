/*
CSCE 236: Embedded Systems
Project 2 Competition
Written by: Sawyer Jager, Rees Klintworth, Derek Nordgren
*/

#include <Servo.h>
#include <Wire.h>

#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read

#define COMMAND_0 0x80  // starts measurments, relays data ready info
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_FREQ 0x89  // Proximity IR test signal freq, 0-3
#define PROXIMITY_MOD 0x8A  // proximity modulator timing

int proximityValue;
int proxHighTolerance = 3000;
int proxLowTolerance = 2950;

int leftBump = 2;
int rightBump = 3;
int numBumps = 0;

boolean panic = false;

int BUTTON = 4;
int RED = 8;
int GREEN = 7;
int BLUE = 6;
int leftSensor = A3;
int rightSensor = A2;
Servo leftWheel;
Servo rightWheel;
int leftDark = 920; //Approxiate dark threshold for left sensor
int rightDark = 910; //pproximate dark threshold for right sensor
float speedFactor;
float turnFactor;
boolean stopMoving = true;
int leftValue;
int rightValue;
boolean crossedBlack;


void setup() {
  leftWheel.attach(9);
  rightWheel.attach(10);
  speedFactor = 1; //Percentage of full speed robot will move straight with
  turnFactor = .5; //Percentage of full speed robot will turn with
  
  Wire.begin();
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
  
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RED, OUTPUT); //Indicates RIGHT turn
  pinMode(GREEN, OUTPUT); //Indicates LEFT turn
  pinMode(BLUE, OUTPUT);  //Indicates STRAIGHT
  pinMode(leftBump, INPUT_PULLUP);
  pinMode(rightBump, INPUT_PULLUP);
  
  digitalWrite(GREEN, HIGH);
  delay(1000);
  digitalWrite(GREEN, LOW);
  
  Serial.begin(9600);
}
  
void loop() {
  resetLights();
  digitalWrite(GREEN, HIGH);
  
  if (digitalRead(BUTTON) == LOW) //Button press will start or stop the robot
  {
    stopMoving = !stopMoving;
    delay(200); //Prevents button bounce
  }
  
  if (digitalRead(leftBump) == LOW) //Enter panic mode if bump sensor is triggered
  {
    panicLeft();
  }
  
  if (digitalRead(rightBump) == LOW) //Enter panic mode if bump sensor is triggered
  {
    panicRight();
  }
    
  if (!stopMoving)
  {    
    proximityValue = readProximity();
    if(proximityValue > proxHighTolerance) //If proximity sensor has detected an object
    {
      avoidObstacle();
    }
    
    leftValue = analogRead(leftSensor);
    rightValue = analogRead(rightSensor);
    resetLights();
    
    if ((leftValue < leftDark - 20) && (rightValue < rightDark - 20)) //Both sensors see white
    {
      if(panic) { //Tries to find the line
        moveStraight();
      } else if (crossedBlack) { //If right sensor has crossed the black line, need to turn right
        turnRight();
      }
      else {
        turnLeft();
      }
    }
    else if ((leftValue > leftDark) && (rightValue < rightDark)) //Only right sensor sees white
    {
      panic = false; //Found the line
      crossedBlack = false;
      moveStraight();
    }
    else if ((leftValue < leftDark) && (rightValue > rightDark)) //Only left sensor sees white
    {
      panic = false; //Found the line
      crossedBlack = true;
      turnRight();
    }
    else //Both sensors see black
    {
      panic = false; //Found the line
      moveStraight();
    }
  }  
  else {
    stopRobot();
    resetLights();
  }
}

//Set of operations that works to get the robot back to the line
void panicLeft(){
  resetLights();
  digitalWrite(RED, HIGH);
  panic = true;
  reverse(); //Back up slightly to clear current path
  delay(1000);  
  turnLeftInPlace(); //Turn left slightly
  delay(100);
  listRight();
  while(digitalRead(rightBump) == HIGH && analogRead(leftSensor) < leftDark){  //While the bump sensor is not triggered, and the IR sensor doesn't see black
    proximityValue = readProximity();
    if(proximityValue > proxHighTolerance)
    {
      turnLeftInPlace(); //Turn left if robot sees an obstacle
      delay(300);
      listRight(); //Very slightly move to the right
    }
  }
  turnLeftInPlace(); //Turn right slightly
  delay(10);
  stopRobot();
}

void panicRight(){
  resetLights();
  digitalWrite(RED, HIGH);
  reverse(); //Back up slightly to clear current path
  panic = true;
  delay(1000);
  turnRightInPlace(); //Turn right slightly
  delay(100);
  listLeft();
  while(digitalRead(leftBump) == HIGH && analogRead(leftSensor) < leftDark){  //While the bump sensor is not triggered, and the IR sensor doesn't see black
    proximityValue = readProximity();
    if(proximityValue > proxHighTolerance)
    {
      turnRightInPlace(); //Turn rught if robot sees an obstacle
      delay(300);
      listLeft(); //Very slight move to the left
    }
  }
  turnRightInPlace(); //Turn right slightly
  delay(10);
}

void avoidObstacle(){
  resetLights();
  digitalWrite(BLUE, HIGH);
  stopRobot();
  turnRightInPlace(); //Turn right to initially avoid obstacle
  delay(400);
  stopRobot();
  proximityValue = readProximity();
  Serial.println(proximityValue);
  if(proximityValue < proxLowTolerance)
  {
    //Avoid  obstacle to the right
    moveStraight();
    delay(850);
    stopRobot();
    numBumps = 1;
    turnLeftInPlace();
    delay(300);
    numBumps = 0;
    turnLeftInPlaceSlow();
    delay(150);
    Serial.println(numBumps);
    //Adjust back to right
    turnRight();
    delay(250);
    findTheLine();
    stopRobot();
  } else {
    // Check left side for clear path
    delay(1000);
    if(digitalRead(rightBump) == LOW){
      panicRight();
    } else {
      panicLeft();
    }
  }
}

void listLeft()
{
  leftWheel.write(110); 
  rightWheel.write(30);
}

void listRight()
{
  leftWheel.write(150);
  rightWheel.write(70);
}

void countBump()
{
  stopRobot();
  detachInterrupt(0);
  numBumps++;
}

//Attempts to get the robot back on the line
void findTheLine(){
  moveStraight();
  delay(750);
  listLeft();
  resetLights();
  digitalWrite(BLUE, HIGH);
  digitalWrite(RED, HIGH);
  while(digitalRead(leftBump) == HIGH && analogRead(leftSensor) < leftDark){ //While the bump sensor is not triggered, and the IR sensor doesn't see black
    proximityValue = readProximity();
    if(proximityValue > proxHighTolerance) //If the robot sees an obstacle
    {
      turnRightInPlace();
      delay(300);
      listLeft();
    }
  };
  delay(10);
  stopRobot();
}

void resetLights() {
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(GREEN, LOW);
}

void stopRobot() {
  leftWheel.write(90);
  rightWheel.write(90);
}

void moveStraight() {
  leftWheel.write(90 + (speedFactor*90)); //Left wheel turns
  rightWheel.write(90 - (speedFactor*90)); //Right wheel turns
}

void reverse() {
  leftWheel.write(90 - (speedFactor*90)); //Left wheel turns backwards
  rightWheel.write(90 + (speedFactor*90)); //Right wheel turns backwards
}

void turnLeft() {
  rightWheel.write(90 - (turnFactor*90)); //Right wheel turns
  leftWheel.write(90); //Left wheel stops
}

void turnRight() {
  leftWheel.write(90 + (turnFactor*90)); //Left wheel turns
  rightWheel.write(90); //Right wheels stops
}

void turnRightInPlace() {
  leftWheel.write(180); //Left wheel turns
  rightWheel.write(180); //Right wheels turns backwards
}

void turnLeftInPlace() {
  leftWheel.write(0); //Left wheel turns backwards
  rightWheel.write(0); //Right wheels turns
}

void turnLeftInPlaceSlow() {
  leftWheel.write(45); //Left wheel turns backwards
  rightWheel.write(45); //Right wheel turns
}

// readProximity() returns a 16-bit value from the VCNL4000's proximity data registers
unsigned int readProximity()
{
  unsigned int data;
  byte temp;
  
  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x08);  // command the sensor to perform a proximity measure
  
  while(!(readByte(COMMAND_0)&0x20)) 
    ;  // Wait for the proximity data ready bit to be set
  data = readByte(PROXIMITY_RESULT_MSB) << 8;
  data |= readByte(PROXIMITY_RESULT_LSB);
  
  return data;
}

// writeByte(address, data) writes a single byte of data to address
void writeByte(byte address, byte data)
{
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

// readByte(address) reads a single byte of data from address
byte readByte(byte address)
{
  byte data;
  
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(VCNL4000_ADDRESS, 1);
  while(!Wire.available())
    ;
  data = Wire.read();

  return data;
}

