// #include <HardwareSerial.h>
// #include <Arduino.h>

// HardwareSerial SerialPort(2);
// const int Enable = 2;
// int receivedData; // Variable to store the received time in seconds
// int h = 0, m = 0, s = 0;

// void setup() {
//   Serial.begin(115200);             // Begin normal serial communication
//   SerialPort.begin(115200, SERIAL_8N1, 16, 17); // Begin RS485 communication
//   pinMode(Enable, OUTPUT);
//   digitalWrite(Enable, LOW);        // Enable RS485 receive mode
// }

// void loop() {
//   digitalWrite(Enable, LOW);  // Set RS485 to receive mode

//   if (SerialPort.available()) {

//     // Receive the data sent from the master
//     receivedData = SerialPort.parseInt();  // Receive the integer time in seconds

//     Serial.print("Received data (seconds): ");
//     Serial.println(receivedData);

//     // Convert received seconds back to hours, minutes, and seconds
//     ConvertIntoHMS(receivedData);

//     // Print the converted time
//     Serial.print("Time: ");
//     Serial.print(h);
//     Serial.print(":");
//     Serial.print(m);
//     Serial.print(":");
//     Serial.println(s);

//     // // Respond back to the master
//     // delay(80)
//     // digitalWrite(Enable, HIGH);  // Enable RS485 transmission
//     // //SerialPort.println("Data received by slave and processed.");
//     // delay(80)

//     digitalWrite(Enable, LOW);   // Set RS485 to receive mode again
//     delay(80);

//   }

//   delay(50); // Small delay for stability
// }

// // // Function to convert seconds into hours, minutes, and seconds
// void ConvertIntoHMS(int totalSeconds) {
//   h = totalSeconds / 3600;        // Get hours
//   totalSeconds %= 3600;
//   m = totalSeconds / 60;          // Get minutes
//   s = totalSeconds % 60;          // Get seconds
// }






// ---- above code is working correctly---- HOAX












// below one was better


// #define MAX_MESG 20


// #include <HardwareSerial.h>
// #include <Arduino.h>

// // Header file includes
// #include <WiFi.h>
// #include <time.h>

// #include <MD_Parola.h>
// #include <SPI.h>

// #include "Font_Data.h"

// #define HARDWARE_TYPE MD_MAX72XX::FC16_HW
// #define MAX_DEVICES 4

// //Main 1 display
// #define CLK_PIN1 18   // or SCK  //yellow
// #define DATA_PIN1 19  // or MOSI  //red
// #define CS_PIN1 5     // or SS //orange

// MD_Parola P_1 = MD_Parola(HARDWARE_TYPE, DATA_PIN1, CLK_PIN1, CS_PIN1, MAX_DEVICES);




// #define SPEED_TIME 75
// #define PAUSE_TIME 0
// #define MAX_MESG 20




// HardwareSerial SerialPort(2);
// const int Enable = 2;
// int receivedData = 0;  // Variable to store the received time in seconds
// uint16_t h = 0, m = 0, s = 0;

// // Global variables
// char szTime[9];    // mm:ss\0
// char szsecond[4];  // ss
// char szMesg[MAX_MESG + 1] = "";
// int i = 0;

// //time data variables
// int recieved_seconds;
// int prevData = 0;
// int count = 0;



// void getsec(char *psz) {
//   if (s > 59) {
//     s = 0;
//   }
//   // uncomment t when integrating with max7219
//   sprintf(psz, "%02d", s);
// }
// void reDisplayP1() {
//   //preacher

//   P_1.begin(3);
//   P_1.setInvert(false);

//   P_1.setZone(0, 0, 0);
//   P_1.setZone(1, 1, 3);
//   P_1.setFont(0, numeric7Seg);
//   P_1.setFont(1, numeric7Se);
//   P_1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
//   P_1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);
//   //getsec(szsecond);
//   //getTime(szTime);
// }

// void getTime(char *psz, bool f = true) {

//   //Serial.println("TIME TO BE PRINTED");
//   sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
//   // Serial.println(psz);

// }


// void buildTime(int recievedSeconds) {
//   m = recievedSeconds / 60;
//   s = recievedSeconds % 60;
//   h = m / 60;
//   m = m % 60;
// }


// void setup() {
//   Serial.begin(115200);                          // Begin normal serial communication
//   SerialPort.begin(115200, SERIAL_8N1, 16, 17);  // Begin RS485 communication
//   pinMode(Enable, OUTPUT);
//   digitalWrite(Enable, LOW);  // Enable RS485 receive mode


//   //  P_0.begin(3);
//   P_1.begin(3);

//   P_1.setInvert(false);
//   // pathi display
//   P_1.setZone(0, 0, 0);
//   P_1.setZone(1, 1, 3);
//   P_1.setFont(0, numeric7Seg);  //seconds
//   P_1.setFont(1, numeric7Se);   //min
//   P_1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
//   P_1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

//   getTime(szTime);
// }

// void loop() {



//   static uint32_t lastTime = 0;         // millis() memory
//   static uint32_t lastTimeRecieve = 0;  // millis() memory
//   static uint32_t lastTimePlus = 0;
//   static uint32_t lastTimeMin = 0;  // millis() memory
//   static uint32_t lastTimeHr = 0;   // millis() memory
//   static uint8_t display = 0;       // current display mode
//   static bool flasher = false;      // seconds passing flasher

//                    P_1.displayAnimate();


//   if (millis() - lastTimeRecieve >= 1000)

//   {
//     digitalWrite(Enable, LOW);  // Set RS485 to receive mode

//     lastTimeRecieve = millis();

//     while (SerialPort.available()) {
//       //Serial.println()


//       //int recieved_seconds = Receiver.parseInt();// Receiver.parseInt(); // Display the Receivers characters

//       int currData = SerialPort.parseInt();
//       if (currData != prevData) {
//         recieved_seconds = currData;
//         recieved_seconds++;
//         buildTime(recieved_seconds);
//         count = 0;
//       } else {
//         count++;
//       }
//       prevData = currData;
//       Serial.println("recieved_seconds");
//       Serial.println(recieved_seconds);
//       //buildTime(recieved_seconds);

//        Serial.print(h);


//        Serial.print(" : ");
//        Serial.print(m);


//        Serial.print(" : ");
//        Serial.print(s);
//     }
//     digitalWrite(Enable, HIGH);  // Set RS485 to transmit  mode
//   }


//   if (millis() - lastTime >= 100 || i == 0) {
//     //buildTime(recieved_seconds);
//     lastTime = millis();
//     getsec(szsecond);
//     getTime(szTime, flasher);
//      P_1.displayReset(0);
//                // P.displayReset(1);
//               P_1.displayReset(1);
//   }

//   if (millis() - lastTimePlus >= 900 || i == 0)

//   {

//     recieved_seconds++;
//     buildTime(recieved_seconds);
//     lastTimePlus = millis();
//   }
//   i = 1;
//   digitalWrite(Enable, LOW);  // Set RS485 to receive mode
//   delay(50);                  // Small delay for stability
// }

// // // Function to convert seconds into hours, minutes, and seconds
// void ConvertIntoHMS(int totalSeconds) {
//   h = totalSeconds / 3600;  // Get hours
//   totalSeconds %= 3600;
//   m = totalSeconds / 60;  // Get minutes
//   s = totalSeconds % 60;  // Get seconds
// }



// Define constants for valid range in 12-hour format
#define MIN_VALID_SECONDS 3600   // Minimum valid value (1:00:00 in seconds)
#define MAX_VALID_SECONDS 46799   // Maximum valid value (12:59:59 in seconds)


//----------------
#include <HardwareSerial.h>
#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <MD_Parola.h>
#include <SPI.h>
#include "Font_Data.h"

// Define constants
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4
#define CLK_PIN1 18   // or SCK  //yellow
#define DATA_PIN1 19  // or MOSI  //red
#define CS_PIN1 5     // or SS //orange
#define SPEED_TIME 75
#define PAUSE_TIME 0
#define MAX_MESG 20

// Instantiate display object
MD_Parola P_1 = MD_Parola(HARDWARE_TYPE, DATA_PIN1, CLK_PIN1, CS_PIN1, MAX_DEVICES);

HardwareSerial SerialPort(2);
const int Enable = 2;
int received_seconds = -1;  // Initialize to -1 to indicate no valid data received yet
uint16_t h = 0, m = 0, s = 0;
char szTime[9];     // mm:ss\0
char szsecond[4];   // ss
int prevData = -1;  // Initialize to -1 to indicate no previous valid data

// Helper functions
void getSecondsDisplay(char *psz) {
  if (s > 59) s = 0;
  sprintf(psz, "%02d", s);
}

void getTimeDisplay(char *psz, bool showColon = true) {
  sprintf(psz, "%02d%c%02d%c", h, (showColon ? ':' : ' '), m, ':');
}

void buildTime(int totalSeconds) {
  m = totalSeconds / 60;
  s = totalSeconds % 60;
  h = m / 60;
  m = m % 60;
}

void initializeDisplay() {
  P_1.begin(3);
  P_1.setInvert(false);
  P_1.setZone(0, 0, 0);
  P_1.setZone(1, 1, 3);
  P_1.setFont(0, numeric7Seg);
  P_1.setFont(1, numeric7Se);
  P_1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);
}

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("I am restarting dude");
  pinMode(Enable, OUTPUT);
  digitalWrite(Enable, LOW);  // Enable RS485 receive mode
  initializeDisplay();
  getTimeDisplay(szTime);
}

void loop() {
  static uint32_t lastTimeUpdate = 0;       // Timer for display updates
  static uint32_t lastTimeReceive = 0;      // Timer for RS485 receive updates
  static uint32_t lastSecondIncrement = 0;  // Timer for second increment

  // Animate display
  P_1.displayAnimate();

  // Receive data every second
  if (millis() - lastTimeReceive >= 1000) {
    digitalWrite(Enable, LOW);  // Set RS485 to receive mode
    lastTimeReceive = millis();

    if (SerialPort.available()) {
      Serial.println("I am available in the prt ");
      int currData = SerialPort.parseInt();
      if (currData >= MIN_VALID_SECONDS && currData <= MAX_VALID_SECONDS && currData != prevData) {
        received_seconds = currData;
        buildTime(received_seconds);
        prevData = currData;
      }

      Serial.println("recieved_seconds");
      Serial.println(received_seconds);
      Serial.print(h);
      Serial.print(" : ");
      Serial.print(m);
      Serial.print(" : ");
      Serial.print(s);


    } else {
      Serial.println("UNAVAILABLE PORT hANG STATE ");
    }
    digitalWrite(Enable, HIGH);  // Set RS485 to transmit mode
  }

  // Update display every 100 milliseconds
  if (millis() - lastTimeUpdate >= 100) {
    lastTimeUpdate = millis();
    getSecondsDisplay(szsecond);
    getTimeDisplay(szTime, true);
    P_1.displayReset(0);
    P_1.displayReset(1);
  }

  // Increment seconds every 900 milliseconds
  if (millis() - lastSecondIncrement >= 900) {

    if (received_seconds >= MIN_VALID_SECONDS && received_seconds <= MAX_VALID_SECONDS) {  // Ignore if `received_seconds` is zero
      received_seconds++;
      buildTime(received_seconds);
    }

    lastSecondIncrement = millis();
  }

  digitalWrite(Enable, LOW);  // Set RS485 to receive mode
  delay(50);                  // Small delay for stability
}
