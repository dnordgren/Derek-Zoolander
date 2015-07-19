#include <Servo.h>
#include <Wire.h>

#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read

#define COMMAND_0 0x80  // starts measurments, relays data ready info
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_FREQ 0x89  // Proximity IR test signal freq, 0-3
#define PROXIMITY_MOD 0x8A  // proximity modulator timing

int red = 8;
int green = 7;
int blue = 6;
int button = 5;
int button2 = 4;

int proximityValue;

int mid = 90;
int offset = 15;

int middle;
int left;
int right;

int i, sum, justRight, tooClose, tooFar;

Servo leftServo;
Servo rightServo;

void setup() {
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(button, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  
  Serial.begin(9600);
  
  leftServo.attach(9);
  rightServo.attach(10);
  
  Wire.begin();
  //writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay

  // blink LEDs
  digitalWrite(red, HIGH);
  delay(250);
  digitalWrite(red, LOW);
  digitalWrite(green, HIGH);
  delay(250);
  digitalWrite(green, LOW);
  digitalWrite(blue, HIGH);
  delay(250);
  digitalWrite(blue, LOW);
  
  // calibrate "too close" value
  digitalWrite(blue, HIGH);
  while(digitalRead(button2) == HIGH);
  for (i = 0, sum = 0; i < 5; i++) {
       sum += readProximity();
  }
  tooClose = sum/5;
  //Serial.println(tooClose, DEC);
  digitalWrite(blue, LOW);
  
  digitalWrite(red, HIGH);
  delay(500);
  digitalWrite(red, LOW);
  
  // calibrate "too far" value
  digitalWrite(blue, HIGH);
  while(digitalRead(button2) == HIGH);
  for(i = 0, sum = 0; i < 5; i++) {
       sum += readProximity();   
  }
  tooFar = sum/5;
  //Serial.println(tooFar, DEC);
  digitalWrite(blue, LOW);
  
  digitalWrite(red, HIGH);
  delay(500);
  digitalWrite(red, LOW);
  
  // calibrate "just right" value
  digitalWrite(blue, HIGH);
  while(digitalRead(button2) == HIGH);
  for(i = 0, sum = 0; i < 5; i++) {
       sum += readProximity();   
  }
  justRight = sum/5;
  //Serial.println(justRight, DEC);
  digitalWrite(blue, LOW);
  
  digitalWrite(red, HIGH);
  delay(500);
  digitalWrite(red, LOW);
  
  digitalWrite(green, HIGH);
  while(digitalRead(button2) == HIGH);
}

void loop() {
  proximityValue = readProximity();
  if(proximityValue < tooFar) {
    // turn left
    leftServo.write(mid - offset);
    rightServo.write(mid - offset);
  } else if (proximityValue > tooClose) {
    // turn right
    leftServo.write(mid + offset);
    rightServo.write(mid + offset);
  } else if (proximityValue > justRight-3 && proximityValue < justRight+3){
    // drive straight
    leftServo.write(180);
    rightServo.write(0);
  }
  
  // Serial.println(proximityValue, DEC);
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

