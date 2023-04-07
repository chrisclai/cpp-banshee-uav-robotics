/*
* Ardupot Receiver Code LiPo RC Car
* Christopher Lai
* BANSHEE UAV Robotics Team
* Last Updated: 9/29/2022
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00501";
const uint8_t  channel = 123;

bool conn = false;

// Pin Declarations
const int motL_1 = 5;
const int motR_1 = 6;
const int motL_2 = 9;
const int motR_2 = 10;

const int sol_1 = A0;
const int sol_2 = A1;

void setup() {
  // Set Up Serial Comms - Debug
  Serial.begin(9600);

  // Set up Radio
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW,0);
  radio.setChannel(channel);
  Serial.println("Starting Radio"); 

  // PinMode Setup
  pinMode(motL_1, OUTPUT);
  pinMode(motR_1, OUTPUT);
  pinMode(motL_2, OUTPUT);
  pinMode(motR_2, OUTPUT);
  pinMode(sol_1, OUTPUT);
  pinMode(sol_2, OUTPUT);

  // Set outputs to 0 on default
  hard_stop();
  analogWrite(sol_1, 0);
  analogWrite(sol_2, 0);

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
    // Packet = [leftstickx, leftsticky, rightstickx, rightsticky, btnleft, btnright, potleft, potright, toggleleft, toggleright]
    byte packet[] = {0,0,0,0,0,0,0,0,0,0};
    radio.read(&packet, sizeof(packet));

    // Will delay code, only use if necessary
    debug(packet);

    // Display data on OLED screen
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
    display.println("BTN: " + String(packet[4]) + " | " + String(packet[5]));
    display.setCursor(0,8);
    display.println("LX: " + String(packet[0]));
    display.setCursor(64,8);
    display.println("LY: " + String(packet[1]));
    display.setCursor(0,16);
    display.println("RX: " + String(packet[2]));
    display.setCursor(64,16);
    display.println("RY: " + String(packet[3]));
    display.setCursor(0,24);
    display.println("PotL: " + String(packet[6]));
    display.setCursor(64,24);
    display.println("PotR: " + String(packet[7]));
    display.display();

    // Do things with the inputs
    if (packet[4] == 0){
      analogWrite(sol_1, 255);
    }
    else {
      analogWrite(sol_1, 0);
    }
    
    if (packet[5] == 0){
      analogWrite(sol_2, 255);
    }
    else {
      analogWrite(sol_2, 0);
    }

    if (packet[1] > 150){
      go_forward();
    }
    else if (packet[1] < 100){
      go_backward();
    }
    else if (packet[2] > 150){
      turn_right();
    }
    else if (packet[2] < 100) {
      turn_left();
    }
    else
    {
      hard_stop();
    }
    
    delay(50);
    
  }
  else {
    Serial.println("NO CONNECTION");
    conn = false;
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
    display.println("BTN: " + String(0) + " | " + String(0));
    display.setCursor(0,8);
    display.println("LX: " + String(0));
    display.setCursor(64,8);
    display.println("LY: " + String(0));
    display.setCursor(0,16);
    display.println("RX: " + String(0));
    display.setCursor(64,16);
    display.println("RX: " + String(0));
    display.setCursor(0,24);
    display.println("PotL: " + String(0));
    display.setCursor(64,24);
    display.println("PotR: " + String(0));
    display.display();

    // Set outputs to 0 if not connected
    hard_stop();
    analogWrite(sol_1, 0);
    analogWrite(sol_2, 0);
  }
}

void debug(byte packet[]) {
      //THIS IS FOR DEBUG
      Serial.print("LX: ");
      Serial.print(packet[0]);
      Serial.print("; LY: ");
      Serial.print(packet[1]);
      Serial.print("; LB: ");
      Serial.print(packet[4]);
      Serial.print("; RX: ");
      Serial.print(packet[2]); 
      Serial.print("; RY: ");
      Serial.print(packet[3]);
      Serial.print("; RB: ");
      Serial.print(packet[5]);
      Serial.println();
      //delay(5);
}

void go_forward(){
  digitalWrite(motL_1, HIGH);
  digitalWrite(motR_1, LOW);
  digitalWrite(motL_2, HIGH);
  digitalWrite(motR_2, LOW);
}

void go_backward(){
  digitalWrite(motL_1, LOW);
  digitalWrite(motR_1, HIGH);
  digitalWrite(motL_2, LOW);
  digitalWrite(motR_2, HIGH);
}

void turn_right(){
  digitalWrite(motL_1, HIGH);
  digitalWrite(motR_1, LOW);
  digitalWrite(motL_2, LOW);
  digitalWrite(motR_2, HIGH);
}

void turn_left(){
  digitalWrite(motL_1, LOW);
  digitalWrite(motR_1, HIGH);
  digitalWrite(motL_2, HIGH);
  digitalWrite(motR_2, LOW);
}

void hard_stop(){
  digitalWrite(motL_1, LOW);
  digitalWrite(motR_1, LOW);
  digitalWrite(motL_2, LOW);
  digitalWrite(motR_2, LOW);
}
