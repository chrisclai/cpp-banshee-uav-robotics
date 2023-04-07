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
int highmap = 1010;

// Define the digital inputs
#define lBTN 9  // Joystick button 1
#define rBTN 10  // Joystick button 2
#define switchL 0 // Left Switch
#define switchR 1 // Right Switch
#define switchMode 5 // Mode Switch

// Packet = [leftstickx, leftsticky, rightstickx, rightsticky, btnleft, btnright, potleft, potright, modeSW, leftSW, rightSW]

/*

packet length = 17

packet[0] = leftstickxL
packet[1] = leftstickxH
packet[2] = leftstickyL
packet[3] = leftstickyH
packet[4] = rightstickxL
packet[5] = rightstickxH
packet[6] = rightstickyL
packet[7] = rightstickyH
packet[8] = btnleft
packet[9] = btnright
packet[10] = potleftL
packet[11] = potleftH
packet[12] = potrightL
packet[13] = potrightH
packet[14] = modeSW
packet[15] = leftSW
packet[16] = rightSW
*/

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

//Make command packet
CMD_Packet packet;

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
  display.println("HYP CTRL");
  display.display(); 
  delay(2500);
  
  //Set Up Controls
  pinMode(lBTN, INPUT_PULLUP);
  pinMode(rBTN, INPUT_PULLUP);
  pinMode(switchL, INPUT_PULLUP);
  pinMode(switchR, INPUT_PULLUP);
  pinMode(switchMode, INPUT_PULLUP);
}

void loop() {

  int tempx = 0;
  int tempy = 0;

  Serial.println(analogRead(A0));
  
  // Read all analog inputs and map them to one Byte value
  //Gather Left Stick Values
  tempx = map(analogRead(A1),0,1023,lowmap,highmap);
  tempy = map(analogRead(A0),0,1023,highmap,lowmap);
  packet.leftstickxL = (byte)tempx;
  packet.leftstickxH = (byte)(tempx>>8);
  packet.leftstickyL = (byte)tempy;
  packet.leftstickyH = (byte)(tempy>>8);
  
  //Gather Right Stick Values
  tempx = map(analogRead(A3),0,1023,lowmap,highmap);
  tempy = map(analogRead(A2),0,1023,highmap,lowmap);
  packet.rightstickxL = (byte)tempx;
  packet.rightstickxH = (byte)(tempx>>8);
  packet.rightstickyL = (byte)tempy;
  packet.rightstickyH = (byte)(tempy>>8);
  
  // Read all digital inputs
  packet.btnleft = digitalRead(lBTN);
  packet.btnright = digitalRead(rBTN);
  packet.modeSW = digitalRead(switchMode);
  packet.leftSW = digitalRead(switchL);
  packet.rightSW = digitalRead(switchR);

  // Read potentiometer values and assign them to packet
  tempx = map(analogRead(A6),0,1023,highmap,lowmap);
  tempy = map(analogRead(A7),0,1023,highmap,lowmap);
  packet.potleftL = (byte)tempx;
  packet.potleftH = (byte)(tempx>>8);
  packet.potrightL = (byte)tempy;
  packet.potrightH = (byte)(tempy>>8);

  // Send the whole data from the structure to the receiver
  radio.write(&packet, sizeof(packet));

  // debug();
  OLED_display();
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
  display.println("LX: " + String(packet.leftstickxL | ((packet.leftstickxH & 0x03) << 8)));
  display.setCursor(0,30);
  display.println("LY: " + String(packet.leftstickyL | ((packet.leftstickyH & 0x03) << 8)));
  display.setCursor(0,40);
  display.println("RX: " + String(packet.rightstickxL | ((packet.rightstickxH & 0x03) << 8)));
  display.setCursor(0,50);
  display.println("RY: " + String(packet.rightstickyL | ((packet.rightstickyH & 0x03) << 8)));
  display.setCursor(64,20);
  display.println("BTNL: " + String(packet.btnleft));
  display.setCursor(64,30);
  display.println("BTNR: " + String(packet.btnright));
  display.setCursor(64,40);
  display.println("POTL: " + String(packet.potleftL | ((packet.potleftH & 0x03) << 8)));
  display.setCursor(64,50);
  display.println("POTR: " + String(packet.potrightL | ((packet.potrightH & 0x03) << 8)));
  display.display();
}