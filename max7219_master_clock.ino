// Header file includes
#include <WiFi.h>
#include <time.h>
#include <MD_Parola.h>
#include <SPI.h>

#include "Font_Data.h"

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4

//Main 1 display 
#define CLK_PIN1 18  // or SCK
#define DATA_PIN1 19 // or MOSI
#define CS_PIN1 5    // or SS

// Part - 2 check
#define CLK_PIN 14  // or SCK
#define DATA_PIN 12 // or MOSI
#define CS_PIN 2    // or SS

//New display added -1
//
#define DATA_PIN2 32
#define CLK_PIN2 33
#define CS_PIN2 25



//New display added -2 
#define CLK_PIN3 26  // or SCK
#define DATA_PIN3 23 // or MOSI
#define CS_PIN3 27   // or SS


#define Sender_Txd_pin 17  //TX2
#define Sender_Rxd_pin 16 //RX2

// Arbitrary output pins
MD_Parola P = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
MD_Parola P1 = MD_Parola(HARDWARE_TYPE, DATA_PIN1, CLK_PIN1, CS_PIN1, MAX_DEVICES);
MD_Parola P2 = MD_Parola(HARDWARE_TYPE, DATA_PIN2, CLK_PIN2, CS_PIN2, MAX_DEVICES);
MD_Parola P3 = MD_Parola(HARDWARE_TYPE, DATA_PIN3, CLK_PIN3, CS_PIN3, MAX_DEVICES);

#define SPEED_TIME 75
#define PAUSE_TIME 0
#define MAX_MESG 20

//#define BUTTON_PIN 15 // GIOP21 pin connected to button //Hour pin button
// #define BUTTON_MIN 14 //minutes pin button
#define BUTTON_SEC 13 // seconds pin button
#define BUTTON_PIN 15 // GIOP21 pin connected to button //Hour pin button
#define BUTTON_MIN 4 // minutes pin button

// RTC
#include <Wire.h>
#include <RTC.h>
static DS1307 RTC;
// SCL D22
// SDA D21



// Sending/Receiving example

//HardwareSerial Sender(2);   // Define a Serial port instance called 'Sender' using serial port 1 UART2




int h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 1, s2 = 1, hours = 01, minutes = 01, seconds = 0;

// Variables will change:;
int lastState = HIGH; // the previous state from the input pin
int lastMin = HIGH;
int lastSec = HIGH;


bool isConnected = false;

int currentState; // the current reading from the input
int currentMin;
int currentSec;

/**********  User Config Setting   ******************************/

// calculate your timezone in seconds,1 hour = 3600 seconds and 5.30Hrs = 19800
const int timezoneinSeconds = 19800;
/***************************************************************/
int dst = 0;
uint16_t h=01, m=01, s=12;

// Global variables
char szTime[9];   // mm:ss\0
char szsecond[4]; // ss
char szMesg[MAX_MESG + 1] = "";

uint8_t h_mode = CLOCK_H12;

void getsec(char *psz)
{
  if(s>59){
    s=0;
  }
    sprintf(psz, "%02d", s);
}

void getTime(char *psz, bool f = true)
{

 if (!RTC.isRunning())
    {
      Serial.println("here lost my way RTC ");
               RTC.setHourMode(h_mode);
               s++;
               if(s>59){
                s=1;
                    if(m==59){
                        m=0;
                        if(h==12){
                            h=1;
                        }else{
                            h++;
                        }
                    }else{
                        m++;
                    }
               }
              RTC.setTime(h,m,s);

       // RTC.setTime(12, 12, 12);
    }

    h = RTC.getHours();
    Serial.print(h);
    m = RTC.getMinutes();
    s = RTC.getSeconds();
    sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
    Serial.println(psz);
    Serial.println("Hey in the loop dude");
    Serial.println(h);
    Serial.println(m);
    Serial.println(s);
}

void reDisplayAllDisplays(){
     P.begin(3);
    P1.begin(3);
    P2.begin(3);
    P3.begin(3);

    
    
    P.setInvert(false);
    P1.setInvert(false);
    P2.setInvert(false);
    P3.setInvert(false);



    P.setZone(0, 0, 0);
    P.setZone(1, 1, 3);
    P.setFont(0, numeric7Seg);
    P.setFont(1, numeric7Se);
    P.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

    // pathi display
    P1.setZone(0, 0, 0);
    P1.setZone(1, 1, 3);
    P1.setFont(0, numeric7Seg);
    P1.setFont(1, numeric7Se);
    P1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

    
    //new display - 1
    P2.setZone(0, 0, 0);
    P2.setZone(1, 1, 3);
    P2.setFont(0, numeric7Seg);
    P2.setFont(1, numeric7Se);
    P2.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P2.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

    //new display - 2
    P3.setZone(0, 0, 0);
    P3.setZone(1, 1, 3);
    P3.setFont(0, numeric7Seg);
    P3.setFont(1, numeric7Se);
    P3.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P3.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

 
}

void setup(void)
{
    Serial.begin(115200);
    //Sender.begin(115200, SERIAL_8N1, Sender_Txd_pin, Sender_Rxd_pin); // Define and start Sender serial port used for sending clock data 

    
    delay(10);
    RTC.begin();



    P.begin(3);
    P1.begin(3);
    P2.begin(3);
    P3.begin(3);

    
    
    P.setInvert(false);
    P1.setInvert(false);
    P2.setInvert(false);
    P3.setInvert(false);



    P.setZone(0, 0, 0);
    P.setZone(1, 1, 3);
    P.setFont(0, numeric7Seg);
    P.setFont(1, numeric7Se);
    P.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

    // pathi display
    P1.setZone(0, 0, 0);
    P1.setZone(1, 1, 3);
    P1.setFont(0, numeric7Seg);
    P1.setFont(1, numeric7Se);
    P1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

    
    //new display - 1
    P2.setZone(0, 0, 0);
    P2.setZone(1, 1, 3);
    P2.setFont(0, numeric7Seg);
    P2.setFont(1, numeric7Se);
    P2.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P2.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

    //new display - 2
    P3.setZone(0, 0, 0);
    P3.setZone(1, 1, 3);
    P3.setFont(0, numeric7Seg);
    P3.setFont(1, numeric7Se);
    P3.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P3.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

    
    
    
    //  delay(100);
    // Hour min and sec button initialisation
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUTTON_MIN, INPUT_PULLUP);
    
    pinMode(BUTTON_SEC, INPUT_PULLUP);

  //pinMode(Sender_Txd_pin, INPUT_PULLUP);
 // pinMode(Sender_Rxd_pin, INPUT_PULLUP);

    getTime(szTime);
    // RTC.setHourMode(h_mode);x

    if (!RTC.isRunning())
    {
         RTC.setHourMode(h_mode);
        RTC.setTime(01, 01, 12);
    }

}
int i = 0;

int ConvertIntoSeconds(int h, int m, int s){
  int ans = h*3600+m*60+s;
  
  return ans;
}

void loop(void)
{


  
    static uint32_t lastTime = 0;    // millis() memory
    static uint32_t lastTimeMin = 0; // millis() memory
    static uint32_t lastTimeHr = 0;  // millis() 
    static uint32_t lastTimeSend = 0;  // millis() memory


    static uint8_t display = 0;  // current display mode
    static bool flasher = false; // seconds passing flasher


    
              if (digitalRead(BUTTON_SEC) == HIGH) {
                Serial.println("hey i am being the hight bioy");
                   //    P.displayClear();
                  P1.displayClear();
                  // digitalWrite(LED_1, LOW); // turn the LED on
                   }
              else {
                Serial.println("hey i am ihte lo w");
            //  P.displayAnimate();
                   P1.displayAnimate();
              //     digitalWrite(LED_1, HIGH); // turn the LED on
                    }
                if (digitalRead(BUTTON_MIN) == HIGH) {
                Serial.println("hey i am being the hight bioy");
                       P.displayClear();
               //      digitalWrite(LED_2, LOW); // turn the LED on
                   }
              else {
                Serial.println("hey i am ihte lo w");
                     P.displayAnimate();
                   //   digitalWrite(LED_2, HIGH); // turn the LED on

                    }

    // if(RTC.getHours()>12){
    //   h=1;
    //   m= RTC.getMinutes(); 
    //   s=RTC.getSeconds();
    //
    //   RTC.setTime(h,m,s);
    // }

    // read the state of the switch/button:
    currentState = digitalRead(BUTTON_PIN);
     currentMin = digitalRead(BUTTON_MIN);
    currentSec = digitalRead(BUTTON_SEC);

    if (lastState == LOW && currentState == HIGH)
    {
        if (millis() - lastTimeHr >= 400)
        {

            lastTimeHr = millis();
            Serial.println("I am in MAIN loop on press button  -------");
            h = RTC.getHours();
            h++;
            if (h > 12)
            {
                h = 1;
            }
            Serial.print(h);

            RTC.setHours(h);
            Serial.println("The state changed from LOW to HIGH");
        }
    }

    if (lastMin == LOW && currentMin == HIGH)
    {
        if (millis() - lastTimeMin >= 400)
        {
            lastTimeMin = millis();
            Serial.println("I am in MAIN loop on press button  ------- MIN LOOp");
            Serial.print(m);
            m = RTC.getMinutes();
            m++;
            Serial.println("After ++ is ");
            if (m > 59)
            {
                m = 0;
            }
            Serial.print(m);
            RTC.setMinutes(m);
        }
    }

    if (lastSec == LOW && currentSec == HIGH)
    {
      
        Serial.println("I am in MAIN loop on press button  -------SECOND");
        s = 0;
        RTC.setSeconds(0);
        Serial.println("The state changed from LOW to HIGH");
    }
    //pilot basis check 
//
//    if (!RTC.isRunning())
//    {
//        Serial.println("Failed RTC---LOOP reset");
//        //   RTC.setHourMode(h_mode);
//
//        RTC.setTime(hours, minutes, seconds);
//    }


 if (!RTC.isRunning())
    {
      Serial.println("here lost my way RTC ");
               RTC.setHourMode(h_mode);
               s++;
               if(s>59){
                s=1;
                    if(m==59){
                        m=0;
                        if(h==12){
                            h=1;
                        }else{
                            h++;
                        }
                    }else{
                        m++;
                    }
               }
              RTC.setTime(h,m,s);

       // RTC.setTime(12, 12, 12);
    }


    lastState = currentState;
    lastMin = currentMin;
    lastSec = currentSec;

 //   P.displayAnimate();
   // P1.displayAnimate();
    
    P2.displayAnimate();
    P3.displayAnimate();

// sending data to slave 
  Serial.println(lastTime);
   
    if (millis() - lastTime >= 100 || i == 0)

    {

   if (digitalRead(BUTTON_SEC) == HIGH) {
                Serial.println("hey i am being the hight bioy");
                       //P.displayClear();
                      P1.displayClear();
                           //  digitalWrite(LED_1, LOW); // turn the LED on
      
                   }
              else {
                Serial.println("hey i am ihte lo w");
                // P.displayReset(0);
               P1.displayReset(0);
               // P.displayReset(1);
              P1.displayReset(1);
                    
                    //digitalWrite(LED_1, HIGH); // turn the LED on


                    }

                      if (digitalRead(BUTTON_MIN) == HIGH) {
                Serial.println("hey i am being the hight bioy");
                       P.displayClear(); 
                      // digitalWrite(LED_2, LOW); // turn the LED on                      
                   }
              else {
                Serial.println("hey i am ihte lo w");
                P.displayReset(0);

                P.displayReset(1);
              //  digitalWrite(LED_2, HIGH); // turn the LED on


                    }
      
  

        lastTime = millis();
        // lastTimeMin=millis();
        getsec(szsecond);
        getTime(szTime, flasher);
        

        P.displayReset(0);
        P1.displayReset(0);
        P.displayReset(1);                     
        P1.displayReset(1);



        P2.displayReset(0);
        P3.displayReset(0);
        P2.displayReset(1);                     
        P3.displayReset(1);



//int sendData = ConvertIntoSeconds(h, m, s);
//      
//      // will use long recievedata = atoi(string) then will convert from milli seconds to hours minutes and second
//      Serial.println("sendData");
//      Serial.println(sendData);
//
//        Sender.print(sendData);                                // Send it to Sender serial port
      
    }


    if (millis() - lastTimeSend >= 3000 || i == 0)

    {

        lastTimeSend = millis();
        // lastTimeMin=millis();
       


  //int sendData = ConvertIntoSeconds(h, m, s);
      
      // will use long recievedata = atoi(string) then will convert from milli seconds to hours minutes and second
      //Serial.println("sendData");
     // Serial.println(sendData);

    //    Sender.print(sendData);                                // Send it to Sender serial port
     // reDisplayAllDisplays();
    }

    
    i = 1;
        //delay(3000);

}
