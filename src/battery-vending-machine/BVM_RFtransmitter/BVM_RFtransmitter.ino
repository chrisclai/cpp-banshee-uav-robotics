/*
* Transmitter Code for RF Moduli
* Christopher Lai
* Last Updated: 8/26/2022
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00501";
const uint8_t  channel = 123;

int lowmap = 10;
int highmap = 250;

// Define the digital inputs
#define lBTN 9  // Joystick button 1
#define rBTN 10  // Joystick button 2

// Packet = [leftstickx, leftsticky, rightstickx, rightsticky, btnleft, btnright, potleft, potright, toggleleft, toggleright]
byte packet[] = {0,0,0,0,0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW,0);
  radio.setChannel(channel);
  radio.stopListening();
  Serial.println("Sending");

  // Check for OLED Availability
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("LOADING...");
  display.setCursor(0, 20);
  display.println("Setting Up");
  display.setCursor(0, 40);
  display.println("RF Moduli");
  display.display(); 
  delay(2500);
  
  //Set Up Controls
  pinMode(lBTN, INPUT_PULLUP);
  pinMode(rBTN, INPUT_PULLUP);
}

void loop() {

  int tempx = 0;
  int tempy = 0;
  
  // Read all analog inputs and map them to one Byte value
  //Gather Left Stick Values
  tempx = map(analogRead(A1),0,1023,lowmap,highmap);
  tempy = map(analogRead(A0),0,1023,highmap,lowmap);
  packet[0] = tempx;
  packet[1] = tempy;
  //Gather Right Stick Values
  tempx = map(analogRead(A3),0,1023,lowmap,highmap);
  tempy = map(analogRead(A2),0,1023,highmap,lowmap);
  packet[2] = tempx;
  packet[3] = tempy; 
  
  // Read all digital inputs
  packet[4] = digitalRead(lBTN);
  packet[5] = digitalRead(rBTN);

  // Read potentiometer values and assign them to packet
  packet[6] = map(analogRead(A6),0,1023,highmap,lowmap);
  packet[7] = map(analogRead(A7),0,1023,highmap,lowmap);

  // Send the whole data from the structure to the receiver
  radio.write(&packet, sizeof(packet));

  // debug();
  OLED_display();
  
  delay(5);
}

void OLED_display()
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(15,0);
  display.println("CONTROLS");
  display.setTextSize(1);
  display.setCursor(0,20);
  display.println("LX: " + String(packet[0]));
  display.setCursor(0,30);
  display.println("LY: " + String(packet[1]));
  display.setCursor(0,40);
  display.println("RX: " + String(packet[2]));
  display.setCursor(0,50);
  display.println("RY: " + String(packet[3]));
  display.setCursor(64,20);
  display.println("BTNL: " + String(packet[4]));
  display.setCursor(64,30);
  display.println("BTNR: " + String(packet[5]));
  display.setCursor(64,40);
  display.println("POTL: " + String(packet[6]));
  display.setCursor(64,50);
  display.println("POTR: " + String(packet[7]));
  display.display();
}

void debug() {
      //THIS IS FOR DEBUG
      int RecLarge= packet[0];
      Serial.print("LX: ");
      Serial.print(RecLarge);
      RecLarge= packet[1];
      Serial.print("; LY: ");
      Serial.print(RecLarge);
      Serial.print("; LB: ");
      Serial.print(packet[4]);
      RecLarge= packet[2];
      Serial.print("; RX: ");
      Serial.print(RecLarge); 
      RecLarge= packet[3];
      Serial.print("; RY: ");
      Serial.print(RecLarge);
      Serial.print("; RB: ");
      Serial.print(packet[5]);
      Serial.print("; POTL: ");
      Serial.print(packet[6]);
      Serial.print("; POTR: ");
      Serial.print(packet[7]);
      Serial.println();
      delay(5);
}
