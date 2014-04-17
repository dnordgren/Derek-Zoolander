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
int offset = 45;

int middle;
int left;
int right;

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
  
  digitalWrite(green, HIGH);
  delay(1000);
  digitalWrite(green, LOW);

  while(digitalRead(button2) == HIGH);

  digitalWrite(red, HIGH);
  delay(1000);
  digitalWrite(red, LOW);
  
}

void loop() {
  proximityValue = readProximity();
  if(proximityValue < 2660) {
    // turn left
    leftServo.write(mid - offset);
    rightServo.write(mid - offset);
  } else if (proximityValue > 2685) {
    // turn right
    leftServo.write(mid + offset);
    rightServo.write(mid + offset);
  } else if (proximityValue > 2665 && proximityValue < 2680){
    // drive straight
    leftServo.write(180);
    rightServo.write(0);
  }
  
  //if (proximityValue
  Serial.println(proximityValue, DEC);
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

