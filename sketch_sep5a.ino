// Header file includes
#include <WiFi.h>
#include <time.h>

#include <MD_Parola.h>
#include <SPI.h>

#include "Font_Data.h"

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4

//Main 1 display
#define CLK_PIN1 18   // or SCK  //yellow
#define DATA_PIN1 19  // or MOSI  //red
#define CS_PIN1 5     // or SS //orange

// not needed as trying to run rs485
// Part - 2 check
// #define CLK_PIN 14  // or SCK  //yellow
// #define DATA_PIN 12 // or MOSI  //red
// #define CS_PIN 2    // or SS   //orange



//New display added -1
//
#define DATA_PIN2 32  //red
#define CLK_PIN2 33   // yellow
#define CS_PIN2 25    //orange
unsigned long debounceDelay = 50;
#include <DS1307.h>
uint16_t year;

DS1307 rtc;


//New display added -2
#define CLK_PIN3 26   // or SCK //yellow
#define DATA_PIN3 23  // or MOSI //red
#define CS_PIN3 27    // or SS   //orange


#define Sender_Txd_pin 17  //TX2
#define Sender_Rxd_pin 16  //RX2

// Arbitrary output pins
//MD_Parola P_0 = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
MD_Parola P_1 = MD_Parola(HARDWARE_TYPE, DATA_PIN1, CLK_PIN1, CS_PIN1, MAX_DEVICES);
MD_Parola P_2 = MD_Parola(HARDWARE_TYPE, DATA_PIN2, CLK_PIN2, CS_PIN2, MAX_DEVICES);
MD_Parola P_3 = MD_Parola(HARDWARE_TYPE, DATA_PIN3, CLK_PIN3, CS_PIN3, MAX_DEVICES);

#define SPEED_TIME 75
#define PAUSE_TIME 0
#define MAX_MESG 20

//#define BUTTON_PIN 15 // GIOP21 pin connected to button //Hour pin button
// #define BUTTON_MIN 14 //minutes pin button
#define BUTTON_SEC 13  // seconds pin button
#define BUTTON_PIN 15  // GIOP21 pin connected to button //Hour pin button
#define BUTTON_MIN 4   // minutes pin button

// RTC
#include <Wire.h>
// SCL D22
// SDA D21



// Sending/Receiving example

//HardwareSerial Sender(2);   // Define a Serial port instance called 'Sender' using serial port 1 UART2

#include <Arduino.h>
#include <string.h>
#include <HardwareSerial.h>
HardwareSerial SerialPort(2);


int slave_1_id = 1;
int slave_2_id = 2;

const int Enable = 2;


//on off display P2
const int buttonPin1 = 0;  // Button connected to pin 0
//const int ledPin1 = 1;     // LED connected to pin 7

//on off display P3
//const int buttonPin2 = 7;  // Button connected to pin 0
//const int ledPin2 = 3;     // LED connected to pin 7

const int reDisplayButton = 1;  // Button connected to pin 0


int h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 1, s2 = 1, hours = 01, minutes = 01, seconds = 0;

// Variables will change:;
int lastState = HIGH;  // the previous state from the input pin
int lastMin = HIGH;
int lastSec = HIGH;

int currentState;  // the current reading from the input
int currentMin;
int currentSec;

/**********  User Config Setting   ******************************/

// calculate your timezone in seconds,1 hour = 3600 seconds and 5.30Hrs = 19800
const int timezoneinSeconds = 19800;
/***************************************************************/
int dst = 0;
uint16_t h = 01, m = 01, s = 12;

// Global variables
char szTime[9];    // mm:ss\0
char szsecond[4];  // ss
char szMesg[MAX_MESG + 1] = "";
uint8_t sec = 01, minute = 01, hour = 01;
uint8_t day, month;
//PCF8574 pcf8574(PCF8574_address);

// void getTime(char *psz, bool f = true) {
//   uint8_t displayHour = h;

//   // Convert to 12-hour format
//   if (displayHour == 0) {
//     displayHour = 12;  // Midnight case
//   } else if (displayHour > 12) {
//     displayHour -= 12;  // Convert 24-hour to 12-hour format
//   }

//   // Format the time string as HH:MM:SS
//   sprintf(psz, "%02d%c%02d%c%02d", displayHour, (f ? ':' : ':'), m, ':', s);

//   // Reset the display for each zone
//   if (P_1.getZoneStatus(1)) {
//     P_1.displayReset(1);
//   }

//   if (P_2.getZoneStatus(1)) {
//     P_2.displayReset(1);
//   }

//   if (P_3.getZoneStatus(1)) {
//     P_3.displayReset(1);
//   }
// }


void getTime(char *psz, bool f = true) {


if(h>12){
  h=1;
        rtc.set(s, m, h, 24, 12, 2014);
}
  // h = RTC.getHours();
  //m = RTC.getMinutes();
  //s = RTC.getSeconds();
  sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');

  //     if(P_0.getZoneStatus(1)){
  //    sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
  //        P_0.displayReset(1);
  //
  //  }

  if (P_1.getZoneStatus(1)) {
    //sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
    P_1.displayReset(1);
  }

  // if (P_2.getZoneStatus(1)) {
  //   //sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
  //   P_2.displayReset(1);
  // }

  if (P_3.getZoneStatus(1)) {
    //sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
    P_3.displayReset(1);
  }
}

//uint8_t h_mode = CLOCK_H12;

void getsec(char *psz) {
  if (s > 59) {
    s = 0;
  }

  //  if(P_0.getZoneStatus(0)){
  //        sprintf(psz, "%02d", s);
  //        P_0.displayReset(0);
  //
  //  }

  if (P_1.getZoneStatus(0)) {
    sprintf(psz, "%02d", s);
    P_1.displayReset(0);
  }

  if (P_2.getZoneStatus(0)) {
    sprintf(psz, "%02d", s);
    P_2.displayReset(0);
  }

  if (P_3.getZoneStatus(0)) {
    sprintf(psz, "%02d", s);
    P_3.displayReset(0);
  }
}
void reDisplayP3() {
  //pathi room
  P_3.begin(3);
  P_3.setInvert(false);

  P_3.setZone(0, 0, 0);
  P_3.setZone(1, 1, 3);
  P_3.setFont(0, numeric7Seg);
  P_3.setFont(1, numeric7Se);
  P_3.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_3.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);
  //    getsec(szsecond);
  //getTime(szTime);
}


void reDisplayP1() {
  //preacher

  P_1.begin(3);
  P_1.setInvert(false);

  P_1.setZone(0, 0, 0);
  P_1.setZone(1, 1, 3);
  P_1.setFont(0, numeric7Seg);
  P_1.setFont(1, numeric7Se);
  P_1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);
  //getsec(szsecond);
  //getTime(szTime);
}

void reDisplayP0() {
  //preacher

  //       P_0.begin(3);
  //    P_0.setInvert(false);
  //
  //    P_0.setZone(0, 0, 0);
  //    P_0.setZone(1, 1, 3);
  //    P_0.setFont(0, numeric7Seg);
  //    P_0.setFont(1, numeric7Se);
  //    P_0.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  //    P_0.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);
  getsec(szsecond);
  getTime(szTime);
}
void reDisplayP2() {
  //pathi
  P_2.begin(3);
  P_2.setInvert(false);

  P_2.setZone(0, 0, 0);
  P_2.setZone(1, 1, 3);
  P_2.setFont(0, numeric7Seg);
  P_2.setFont(1, numeric7Se);
  P_2.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_2.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);
  //    getsec(szsecond);
  //getTime(szTime);
}

void reDisplayAllDisplays() {
  // P_0.begin(3);
  P_1.begin(3);
  P_2.begin(3);
  P_3.begin(3);

  // P_0.setIntensity(10);


  //  P_0.setInvert(false);
  P_1.setInvert(false);
  P_2.setInvert(false);
  P_3.setInvert(false);



  //    P_0.setZone(0, 0, 0);
  //    P_0.setZone(1, 1, 3);
  //    P_0.setFont(0, numeric7Seg);
  //    P_0.setFont(1, numeric7Se);
  //    P_0.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  //    P_0.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

  // pathi display
  P_1.setZone(0, 0, 0);
  P_1.setZone(1, 1, 3);
  P_1.setFont(0, numeric7Seg);
  P_1.setFont(1, numeric7Se);
  P_1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);


  //new display - 1
  P_2.setZone(0, 0, 0);
  P_2.setZone(1, 1, 3);
  P_2.setFont(0, numeric7Seg);
  P_2.setFont(1, numeric7Se);
  P_2.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_2.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

  //new display - 2
  P_3.setZone(0, 0, 0);
  P_3.setZone(1, 1, 3);
  P_3.setFont(0, numeric7Seg);
  P_3.setFont(1, numeric7Se);
  P_3.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_3.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);
}

//#define RS485_DE_RE_PIN 2


// **Inserted Snippet to Clear SerialPort Buffer Before Sending Data**
void clearSerialBuffer() {
  unsigned long startTime = millis();
  while (SerialPort.available() && millis() - startTime < 100) {
    SerialPort.read();  // Clear any remaining bytes in the buffer
  }
}


void setup(void) {

  //pinMode(RS485_DE_RE_PIN, OUTPUT);




  //
  rtc.begin();
  rtc.start();
  rtc.get(&sec, &minute, &hour, &day, &month, &year);

  // pinMode(RS485_DE_RE_PIN, OUTPUT);
  //digitalWrite(RS485_DE_RE_PIN, HIGH); // Enable transmission
  Serial.begin(115200);

  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(Enable, OUTPUT);
  digitalWrite(Enable, LOW);

  Serial.println("Hey restarting restarting");

  Wire.begin();
  //  P_0.begin(3);
  P_1.begin(3);
  P_2.begin(3);
  P_3.begin(3);


  // P_0.setInvert(false);
  P_1.setInvert(false);
  P_2.setInvert(false);
  P_3.setInvert(false);



  //    P_0.setZone(0, 0, 0);
  //    P_0.setZone(1, 1, 3);
  //    P_0.setFont(0, numeric7Seg);
  //    P_0.setFont(1, numeric7Se);
  //    P_0.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  //    P_0.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

  // pathi display
  P_1.setZone(0, 0, 0);
  P_1.setZone(1, 1, 3);
  P_1.setFont(0, numeric7Seg);
  P_1.setFont(1, numeric7Se);
  P_1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);


  //new display - 1
  P_2.setZone(0, 0, 0);
  P_2.setZone(1, 1, 3);
  P_2.setFont(0, numeric7Seg);
  P_2.setFont(1, numeric7Se);
  P_2.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_2.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

  //new display - 2
  P_3.setZone(0, 0, 0);
  P_3.setZone(1, 1, 3);
  P_3.setFont(0, numeric7Seg);
  P_3.setFont(1, numeric7Se);
  P_3.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_3.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);




  //  delay(100);
  // Hour min and sec button initialisation
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MIN, INPUT_PULLUP);
  pinMode(BUTTON_SEC, INPUT_PULLUP);

  //on off display button with redisplay buttonand led initialisation using gpio expanders


  getTime(szTime);
}
int i = 0;


void sendDataToSlaves() {
  int sendData = ConvertIntoSeconds(h, m, s);
// need to remove the below code if facing any issues due to clearSerialBuffer
clearSerialBuffer();
  // Switch to transmission mode

  delay(10);
  digitalWrite(Enable, HIGH);

  // Send the data to slaves
  //SerialPort.println(sendData);

  // now using print instead of println
  SerialPort.print(sendData);

  Serial.print("Sending data to slaves: ");
  Serial.print(sendData);

  // Switch back to receive mode
  delay(10);
  digitalWrite(Enable, LOW);
  //delay(50);
}



int ConvertIntoSeconds(int h, int m, int s) {
  int ans = h * 3600 + m * 60 + s;

  return ans;
}
int countOn = 0;
int countOff = 0;
void loop(void) {

  rtc.get(&sec, &minute, &hour, &day, &month, &year);

  h = hour;
  m = minute;
  s = sec;
  static uint32_t lastTime = 0;          // millis() memory
  static uint32_t lastTimeMin = 0;       // millis() memory
  static uint32_t lastTimeHr = 0;        // millis()
  static uint32_t lastTimeSend = 0;      // millis() memory
  static uint32_t lastTimeSendP0 = 0;    // millis() memory
  static uint32_t lastTimeSendData = 0;  // millis() memory

  static uint32_t lastTimeSendP2 = 0;  // millis() memory



  static uint8_t display = 0;   // current display mode
  static bool flasher = false;  // seconds passing flasher


  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_PIN);
  currentMin = digitalRead(BUTTON_MIN);
  currentSec = digitalRead(BUTTON_SEC);




  if (lastState == LOW && currentState == HIGH) {
    if (millis() - lastTimeHr >= 400) {

      lastTimeHr = millis();
      //  h = now.hour();
      h++;
      if (h > 12) {
        h = 1;
      }
      rtc.set(s, m, h, 24, 12, 2014);
      Serial.println("The state changed from LOW to HIGH");
    }
  }

  if (lastMin == LOW && currentMin == HIGH) {
    if (millis() - lastTimeMin >= 400) {
      lastTimeMin = millis();
      Serial.println("I am in MAIN loop on press button  ------- MIN LOOp");
      Serial.print(m);
      //m = now.minute();
      m++;
      Serial.println("After ++ is ");
      if (m > 59) {
        m = 0;
      }
      Serial.print(m);
      rtc.set(s, m, h, 24, 12, 2014);


      //            RTC.setMinutes(m);
    }
  }

  if (lastSec == LOW && currentSec == HIGH) {

    Serial.println("I am in MAIN loop on press button  -------SECOND");
    s = 0;
    rtc.set(s, m, h, 24, 12, 2014);
    Serial.println("The state changed from LOW to HIGH");
  }


  lastState = currentState;
  lastMin = currentMin;
  lastSec = currentSec;


  // sending data to slave
  if (millis() - lastTime >= 100 || i == 0)

  {

    P_1.displayAnimate();
    //delay(10);
    //P_0.displayAnimate();
    // delay(10);
    //P_2.displayAnimate();
    //delay(10);
    P_3.displayAnimate();
    //delay(10);

    //int buttonState1= digitalRead(buttonPin1);



    lastTime = millis();
    // lastTimeMin=millis();
    getsec(szsecond);
    getTime(szTime, flasher);
  }

  if (millis() - lastTimeSendData >= 3000 || i == 0)

  {
    Serial.println("sending data to slaves in 3 seconds ");

    lastTimeSendData = millis();
    
    //enable it for master slave clock 
    sendDataToSlaves();  // Call the function to send time data via RS485



    Serial.println("sendData");
  }

  i = 1;
}