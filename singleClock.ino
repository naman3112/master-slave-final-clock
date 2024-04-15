// Header file includes
#include <WiFi.h>
#include <time.h>

#include <MD_Parola.h>
#include <SPI.h>

#include "Font_Data.h"

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4

//Main 1 display 
#define CLK_PIN1 18  // or SCK  //yellow
#define DATA_PIN1 19 // or MOSI  //red 
#define CS_PIN1 5    // or SS //orange

// Part - 2 check
#define CLK_PIN 14  // or SCK  //yellow
#define DATA_PIN 12 // or MOSI  //red
#define CS_PIN 2    // or SS   //orange

//New display added -1
//
#define DATA_PIN2 32 //red 
#define CLK_PIN2 33 // yellow
#define CS_PIN2 25 //orange
unsigned long debounceDelay = 50;    
#include "RTClib.h"
uint16_t year;

RTC_DS1307 rtc;


//New display added -2 
#define CLK_PIN3 26  // or SCK //yellow
#define DATA_PIN3 23 // or MOSI //red
#define CS_PIN3 27   // or SS   //orange

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
#define BUTTON_SEC 13 // seconds pin button
#define BUTTON_PIN 15 // GIOP21 pin connected to button //Hour pin button
#define BUTTON_MIN 4 // minutes pin button

// RTC
#include <Wire.h>
// SCL D22
// SDA D21

// Pins on PCF8574

//on off display P2
const int buttonPin1 = 0;  // Button connected to pin 0


const int reDisplayButton = 1;  // Button connected to pin 0


int h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 1, s2 = 1, hours = 01, minutes = 01, seconds = 0;

// Variables will change:;
int lastState = HIGH; // the previous state from the input pin
int lastMin = HIGH;
int lastSec = HIGH;

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
uint8_t sec, minute, hour, day, month;

//PCF8574 pcf8574(PCF8574_address);

void getTime(char *psz, bool f = true)
{


   // h = RTC.getHours();
    Serial.print(h);
    //m = RTC.getMinutes();
    //s = RTC.getSeconds();
    sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');

  if(P_1.getZoneStatus(1)){
    //sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
           P_1.displayReset(1);


  }
    
  if(P_2.getZoneStatus(1)){
    //sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
                P_2.displayReset(1);


  }
    
  if(P_3.getZoneStatus(1)){
    //sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
                P_3.displayReset(1);


  }
    
    Serial.println(psz);
    Serial.println("Hey in the loop dude");
    Serial.println(h);
    Serial.println(m);
    Serial.println(s);
}

//uint8_t h_mode = CLOCK_H12;

void getsec(char *psz)
{
  if(s>59){
    s=0;
  }
  

  if(P_1.getZoneStatus(0)){
        sprintf(psz, "%02d", s);
           P_1.displayReset(0);


  }
    
  if(P_2.getZoneStatus(0)){
        sprintf(psz, "%02d", s);
                P_2.displayReset(0);


  }
    
  if(P_3.getZoneStatus(0)){
        sprintf(psz, "%02d", s);
                P_3.displayReset(0);


  }
}


void setup(void)
{
//


    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    }

    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }



    // rtc.get(&sec, &minute, &hour, &day, &month, &year);
    DateTime now = rtc.now();
sec = now.second();
minute = now.minute();
hour = now.hour();
day = now.day();
month = now.month();
year = now.year();


    Serial.begin(9600);
  Wire.begin();

    P_1.begin(3);
    P_2.begin(3);
   P_3.begin(3);



   // P_0.setInvert(false);
    P_1.setInvert(false);
    P_2.setInvert(false);
    P_3.setInvert(false);


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


int ConvertIntoSeconds(int h, int m, int s){
  int ans = h*3600+m*60+s;

  return ans;
}
int countOn=0;
int countOff=0;
void loop(void)
{
//  P_0.displayClear();  
     //P_0.displayClear();
    //DateTime now = rtc.now();
DateTime now = rtc.now();
sec = now.second();
minute = now.minute();
hour = now.hour();
day = now.day();
month = now.month();
year = now.year();


 h= hour;
 m=minute;
 s = sec;



    // Convert 24-hour time to 12-hour time with AM/PM
    int displayHour = h;
    bool isPM = false;

    if (displayHour == 0) { // Handle midnight
        displayHour = 12;
        h=12;
    } else if (displayHour == 12) { // Handle noon
        isPM = true;
    
    } else if (displayHour > 12) {
        displayHour -= 12;
        h-=12;
        isPM = true;
    }

    static uint32_t lastTime = 0;    // millis() memory
    static uint32_t lastTimeMin = 0; // millis() memory
    static uint32_t lastTimeHr = 0;  // millis() 
    static uint32_t lastTimeSend = 0;  // millis() memory
        static uint32_t lastTimeSendP0 = 0;  // millis() memory
        static uint32_t lastTimeSendData = 0;  // millis() memory

    static uint32_t lastTimeSendP2 = 0;  // millis() memory



    static uint8_t display = 0;  // current display mode
    static bool flasher = false; // seconds passing flasher


    // read the state of the switch/button:
    currentState = digitalRead(BUTTON_PIN);
     currentMin = digitalRead(BUTTON_MIN);
    currentSec = digitalRead(BUTTON_SEC);
         
        
                    
       Serial.println("goog");

    if (lastState == LOW && currentState == HIGH)
    {
        if (millis() - lastTimeHr >= 400)
        {

            lastTimeHr = millis();
            Serial.println("I am in MAIN loop on press button  -------");
          //  h = now.hour();
            h++;
            if (h > 12)
            {
                h = 1;
            }
            Serial.print(h);
rtc.adjust(DateTime(2014, 12, 24, h, m, s));
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
            //m = now.minute();
            m++;
            Serial.println("After ++ is ");
            if (m > 59)
            {
                m = 0;
            }
            Serial.print(m);
rtc.adjust(DateTime(2014, 12, 24, h, m, s));


//            RTC.setMinutes(m);
        }
    }

    if (lastSec == LOW && currentSec == HIGH)
    {

        Serial.println("I am in MAIN loop on press button  -------SECOND");
        s = 0;
rtc.adjust(DateTime(2014, 12, 24, h, m, s));
        Serial.println("The state changed from LOW to HIGH");
    }



    lastState = currentState;
    lastMin = currentMin;
    lastSec = currentSec;

   
// sending data to slave 
  Serial.println(lastTime);

    if (millis() - lastTime >= 100 || i == 0)

    {  
     
 P_1.displayAnimate();
 //delay(10);
  //P_0.displayAnimate();
 // delay(10);
   P_2.displayAnimate();
   //delay(10);
   P_3.displayAnimate();
   //delay(10);


        lastTime = millis();
        // lastTimeMin=millis();
        getsec(szsecond);
        getTime(szTime, flasher);
                                    

    }

        if (millis() - lastTimeSendData >= 3000 || i == 0)

    {

        lastTimeSendData = millis();


    }


    i = 1;
        //delay(3000);

}
