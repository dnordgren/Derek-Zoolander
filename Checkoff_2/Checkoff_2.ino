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
  
  pinMode(leftBump, INPUT_PULLUP);
  pinMode(rightBump, INPUT_PULLUP);
  
  Wire.begin();
  //writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
  
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RED, OUTPUT); //Indicates RIGHT turn
  pinMode(GREEN, OUTPUT); //Indicates LEFT turn
  pinMode(BLUE, OUTPUT);  //Indicates STRAIGHT
  
  digitalWrite(GREEN, HIGH);
  delay(1000);
  digitalWrite(GREEN, LOW);
  
  Serial.begin(9600);
  /*
  digitalWrite(RED, HIGH);
  while(analogRead(rightSensor) < 950){}
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  while(digitalRead(BUTTON) == HIGH){Serial.println(analogRead(rightSensor));}
  int leftWhite = analogRead(leftSensor);
  int midBlack = analogRead(rightSensor);
  digitalWrite(GREEN, LOW);
  delay(100); 
  Serial.println(midBlack);
  
  digitalWrite(RED, HIGH);
  while (analogRead(leftSensor) < 975){}
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  while(digitalRead(BUTTON) == HIGH){Serial.println(analogRead(leftSensor));}
  int leftBlack = analogRead(leftSensor);
  int midWhite = analogRead(rightSensor);
  digitalWrite(GREEN, LOW);
  
  Serial.print("left");
  Serial.println(leftBlack);
  Serial.print("right");
  Serial.println(midBlack);
  */
  //leftDark = leftBlack-20;
  //rightDark = midBlack-20;  
}
  
void loop() {
  resetLights();
  digitalWrite(GREEN, HIGH);
  if (digitalRead(BUTTON) == LOW) //Button press will start or stop the robot
  {
    stopMoving = !stopMoving;
    delay(200); //Prevents button bounce
  }
  
  if (digitalRead(leftBump) == LOW)
  {
    panicLeft();
  }
  
  if (digitalRead(rightBump) == LOW)
  {
    panicRight();
  }
    
  if (!stopMoving)
  {    
    proximityValue = readProximity();
    if(proximityValue > proxHighTolerance)
    {
      avoidObstacle();
    }
    
    leftValue = analogRead(leftSensor);
    rightValue = analogRead(rightSensor);
    resetLights();
    
    if ((leftValue < leftDark - 20) && (rightValue < rightDark - 20)) //Both sensors see white
    {
      if(panic) {
        moveStraight();
      } else if (crossedBlack) { //If right sensor has crossed the black line, need to turn right
        //digitalWrite(RED, HIGH);
        turnRight();
      }
      else {
        //digitalWrite(GREEN, HIGH);
        turnLeft();
      }
    }
    else if ((leftValue > leftDark) && (rightValue < rightDark)) //Only right sensor sees white
    {
      panic = false;
      //digitalWrite(BLUE, HIGH);
      crossedBlack = false;
      moveStraight();
    }
    else if ((leftValue < leftDark) && (rightValue > rightDark)) //Only left sensor sees white
    {
      panic = false;
      //digitalWrite(GREEN, HIGH);
      crossedBlack = true;
      turnRight();
    }
    else //Both sensors see black
    {
      panic = false;
      //digitalWrite(GREEN, HIGH);
      moveStraight();
    }
  }  
  else {
    stopRobot();
    resetLights();
  }
}

void panicLeft(){
  resetLights();
  digitalWrite(RED, HIGH);
  Serial.println("PANIC (left)");
  panic = true;
  reverse();
  delay(1000);  
  turnLeftInPlace();
  delay(100);
  listRight();
  while(digitalRead(rightBump) == HIGH && analogRead(leftSensor) < leftDark){
    proximityValue = readProximity();
    if(proximityValue > proxHighTolerance)
    {
      turnLeftInPlace();
      delay(300);
      listRight();
    }
  }
  turnLeftInPlace();
  delay(10);
  stopRobot();
}

void panicRight(){
  resetLights();
  digitalWrite(RED, HIGH);
  Serial.println("PANIC (right)");
  reverse();
  panic = true;
  delay(1000);
  turnRightInPlace();
  delay(100);
  listLeft();
  while(digitalRead(leftBump) == HIGH && analogRead(leftSensor) < leftDark){
    proximityValue = readProximity();
    if(proximityValue > proxHighTolerance)
    {
      turnRightInPlace();
      delay(300);
      listLeft();
    }
  }
  turnRightInPlace();
  delay(10);
}

void avoidObstacle(){
  resetLights();
  digitalWrite(BLUE, HIGH);
  stopRobot();
  //delay(1000);
  turnRightInPlace();
  Serial.println("Here");
  //digitalWrite(RED, HIGH);
  delay(400);
  //digitalWrite(RED, LOW);
  stopRobot();
  proximityValue = readProximity();
  Serial.println(proximityValue);
  //resetLights();
  if(proximityValue < proxLowTolerance)
  {
    //Avoid  obstacle to the right
    moveStraight();
    delay(850);
    stopRobot();
    //delay(800);
    numBumps = 1;
    turnLeftInPlace();
    delay(300);
    numBumps = 0;
    //attachInterrupt(0, countBump, FALLING);
    turnLeftInPlaceSlow();
    delay(150);
    Serial.println(numBumps);
    //digitalWrite(GREEN, HIGH);
    //stopRobot();
    //delay(1000);
    //Adjust back to right
    turnRight();
    delay(250);
    //digitalWrite(GREEN, LOW);
    findTheLine();
    stopRobot();
  } else {
    // Check left side for clear path
    //digitalWrite(BLUE, HIGH);
    delay(1000);
    //digitalWrite(BLUE, LOW);
    if(digitalRead(rightBump) == LOW){
      panicRight();
    } else {
      panicLeft();
    }
  }
  //delay(2000);
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

void findTheLine(){
  moveStraight();
  delay(750);
  listLeft();
  resetLights();
  digitalWrite(BLUE, HIGH);
  digitalWrite(RED, HIGH);
  while(digitalRead(leftBump) == HIGH && analogRead(leftSensor) < leftDark){
    proximityValue = readProximity();
    if(proximityValue > proxHighTolerance)
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
  leftWheel.write(90 + (speedFactor*90));
  rightWheel.write(90 - (speedFactor*90));
}

void reverse() {
  leftWheel.write(0);
  rightWheel.write(180);
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
  leftWheel.write(45);
  rightWheel.write(45);
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

