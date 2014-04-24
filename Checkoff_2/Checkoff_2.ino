#include <Servo.h>

int BUTTON = 4;
int RED = 8;
int GREEN = 7;
int BLUE = 6;
int leftSensor = A3;
int rightSensor = A2;
Servo leftWheel;
Servo rightWheel;
int leftDark = 940; //Approxiate dark threshold for left sensor
int rightDark = 920; //pproximate dark threshold for right sensor
float speedFactor;
float turnFactor;
boolean stopMoving = true;
int leftValue;
int rightValue;
boolean crossedBlack;


void setup() {
  leftWheel.attach(9);
  rightWheel.attach(10);
  speedFactor = .75; //Percentage of full speed robot will move straight with
  turnFactor = .5; //Percentage of full speed robot will turn with
  
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RED, OUTPUT); //Indicates RIGHT turn
  pinMode(GREEN, OUTPUT); //Indicates LEFT turn
  pinMode(BLUE, OUTPUT);  //Indicates STRAIGHT
  
  Serial.begin(9600);
}
  
void loop() {
  
  if (digitalRead(BUTTON) == LOW) //Button press will start or stop the robot
  {
    stopMoving = !stopMoving;
    delay(200); //Prevents button bounce
  }
  
  if (!stopMoving)
  {
    leftValue = analogRead(leftSensor);
    rightValue = analogRead(rightSensor);
    resetLights();
    
    if ((leftValue < leftDark) && (rightValue < rightDark)) //Both sensors see white
    {
      if (crossedBlack) { //If right sensor has crossed the black line, need to turn right
        digitalWrite(RED, HIGH);
        turnRight();
      }
      else {
        digitalWrite(GREEN, HIGH);
        turnLeft();
      }
    }
    else if ((leftValue > leftDark) && (rightValue < rightDark)) //Only right sensor sees white
    {
      digitalWrite(BLUE, HIGH);
      crossedBlack = false;
      moveStraight();
    }
    else if ((leftValue < leftDark) && (rightValue > rightDark)) //Only left sensor sees white
    {
      digitalWrite(GREEN, HIGH);
      crossedBlack = true;
      turnRight();
    }
    else //Both sensors see black
    {
      digitalWrite(GREEN, HIGH);
      turnRight();
    }
  }  
  else {
    stopRobot();
    resetLights();
  }
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

void turnLeft() {
  rightWheel.write(90 - (turnFactor*90)); //Right wheel turns
  leftWheel.write(90); //Left wheel stops
}

void turnRight() {
  leftWheel.write(90 + (turnFactor*90)); //Left wheel turns
  rightWheel.write(90); //Right wheels stops
}

