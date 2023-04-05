/*
* Ardupot Receiver Code for Hyperloop Pod Control
* Christopher Lai
* BANSHEE UAV
* Last Updated: 4/4/2023
*
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "PPMEncoder.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00501";
const uint8_t  channel = 123;

const int echoPin1 = A0;
const int trigPin1 = A1;
const int echoPin2 = A2;
const int trigPin2 = A3;

const int dir1 = 5;
const int pwm1 = 6;
const int dir2 = 9;
const int pwm2 = 10;

long duration = 0;
int distanceL = 0;
int distanceR = 0;
bool conn = false;

// Max size 32 bytes because of buffer limit
struct CMD_Packet {
  byte leftstickxL;
  byte leftstickxH;
  byte leftstickyL;
  byte leftstickyH;
  byte rightstickxL;
  byte rightstickxH;
  byte rightstickyL;
  byte rightstickyH;
  byte btnleft;
  byte btnright;
  byte potleftL;
  byte potleftH;
  byte potrightL;
  byte potrightH;
  byte modeSW;
  byte leftSW;
  byte rightSW;
};

// Simplified Packet
struct CMD_Tran{
  byte LButton;
  byte RButton;
  int LStickY;
  int LStickX;
  int RStickX;
  int RStickY;
  int LTrim;
  int RTrim;
  byte BbyMode;
  byte leftSW;
  byte rightSW;
};

CMD_Packet packet;
CMD_Tran cmd;

void setup() {
  // Set Up Serial Comms - Debug
  Serial.begin(9600);

  // Pinmode Setup
  pinMode(echoPin1, INPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin2, OUTPUT);

  // Force stop
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);

  // Set up Radio
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW,0);
  radio.setChannel(channel);
  Serial.println("Starting Radio"); 

  // Check for OLED Availability
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  //to recieve
  radio.startListening();
}

void loop() {  
  if (radio.available()) {
    conn = true;
    radio.read(&packet, sizeof(packet));

    int RecLarge = packet.leftstickxL | ((packet.leftstickxH&0x03)<<8);
    cmd.LStickX = RecLarge;
    
    RecLarge= packet.leftstickyL | ((packet.leftstickyH&0x03)<<8);  
    cmd.LStickY = RecLarge;
    
    RecLarge= packet.rightstickxL | ((packet.rightstickxH&0x03)<<8);
    cmd.RStickX = RecLarge;
    
    RecLarge= packet.rightstickyL | ((packet.rightstickyH&0x03)<<8);
    cmd.RStickY = RecLarge;
    
    RecLarge= packet.potleftL | ((packet.potleftH&0x03)<<8);
    cmd.LTrim = RecLarge;
    
    RecLarge = packet.potrightL | ((packet.potrightH&0x03)<<8); 
    cmd.RTrim = RecLarge;

    cmd.LButton = packet.btnleft;
    cmd.RButton = packet.btnright;
    cmd.BbyMode = packet.modeSW;
    cmd.leftSW = packet.leftSW;
    cmd.rightSW = packet.rightSW;
    
    OLED_display();
    drivePod();
    toRPi();

    delay(5);
  }
  else{
    OLED_display();
    drivePod();
    conn = false;
    toRPi();
    resetCMD();
  }
}

void OLED_display(){
  if (cmd.BbyMode){
    OLED_display_MAIN();
  }
  else
  {
    OLED_display_POD();
  }
}

void OLED_display_MAIN() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  if (conn)
  {
    display.println("RF: YES");
  }
  else
  {
    display.println("RF: NO");
  }
  
  display.setCursor(64,0);
  display.println("BTN: " + String(cmd.LButton) + " | " + String(cmd.RButton));
  display.setCursor(0,8);
  display.println("LX: " + String(cmd.LStickX));
  display.setCursor(64,8);
  display.println("RX: " + String(cmd.RStickX));
  display.setCursor(0,16);
  display.println("LY: " + String(cmd.LStickY));
  display.setCursor(64,16);
  display.println("RY: " + String(cmd.RStickY));
  display.setCursor(0,24);
  display.println("LPOT: " + String(cmd.LTrim));
  display.setCursor(64,24);
  display.println("RPOT: " + String(cmd.RTrim));
  display.display();
}

void toRPi() {
  String output = "";
  output += String(cmd.LStickX) + " "; 
  output += String(cmd.LStickY) + " "; 
  output += String(cmd.RStickX) + " "; 
  output += String(cmd.RStickY) + " "; 
  output += String(cmd.LTrim) + " "; 
  output += String(cmd.RTrim) + " ";
  output += String(cmd.LButton) + " "; 
  output += String(cmd.RButton) + " ";
  output += String(cmd.BbyMode) + " ";
  output += String(cmd.leftSW) + " "; 
  output += String(cmd.rightSW) + " ";
  Serial.println(output);
}

void drivePod(){
  distanceL = getDistance(trigPin1, echoPin1);
  distanceR = getDistance(trigPin2, echoPin2);  

  analogWrite(pwm1, 120); // left pot
  analogWrite(pwm2, 120);

  if (distanceL <= 15 && distanceR > 15)
  {
    motor_forward(100);
  }
  else if (distanceR <= 20 && distanceL > 20)
  {
    motor_backward(100);
  }
  else if (cmd.LStickX <= 400)
  {
    motor_forward(100);
  }
  else if (cmd.LStickX >= 600)
  {
    motor_backward(100);
  }
  else
  {
    motor_stop();
  }
}

void motor_stop()
{
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
}

void motor_forward(int speed_val)
{
  analogWrite(pwm1, speed_val);
  analogWrite(pwm2, speed_val);
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, HIGH);
}

void motor_backward(int speed_val)
{
  analogWrite(pwm1, speed_val);
  analogWrite(pwm2, speed_val);
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
}

int getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // 0.034 constant from speed of sound
}

void OLED_display_POD()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  if (conn)
  {
    display.println("RF Status: PASS");
  }
  else
  {
    display.println("RF Status: FAIL");
  }
  display.setCursor(0,10);
  display.println("Driver Mode: MAIN");
  display.setCursor(0,20);
  display.println("DistL: " + String(distanceL));
  display.setCursor(64,20);
  display.println("DistR: " + String(distanceR));
  display.display();
}

void resetCMD() {
  cmd.LButton=1;
  cmd.RButton=1;
  cmd.LStickY=512;
  cmd.LStickX=512;
  cmd.RStickX=512;
  cmd.RStickY=512;
  cmd.LTrim=512;
  cmd.RTrim=512;
}
