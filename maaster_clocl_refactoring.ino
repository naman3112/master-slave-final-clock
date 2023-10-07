// Header file includes
#include <WiFi.h>
#include <time.h>
#include "PCF8574.h"

#include <MD_Parola.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(0x10);

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



//New display added -2 
#define CLK_PIN3 26  // or SCK //yellow
#define DATA_PIN3 23 // or MOSI //red
#define CS_PIN3 27   // or SS   //orange


#define Sender_Txd_pin 17  //TX2
#define Sender_Rxd_pin 16 //RX2

// Arbitrary output pins
MD_Parola P_0 = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
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
#include <RTC.h>
static DS1307 RTC;
// SCL D22
// SDA D21



// Sending/Receiving example

//HardwareSerial Sender(2);   // Define a Serial port instance called 'Sender' using serial port 1 UART2



const int PCF8574_address = 0x20;  // Adjust the address if needed
// Pins on PCF8574

//on off display P2
const int buttonPin1 = 0;  // Button connected to pin 0
//const int ledPin1 = 1;     // LED connected to pin 7

//on off display P3
//const int buttonPin2 = 7;  // Button connected to pin 0
//const int ledPin2 = 3;     // LED connected to pin 7

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

PCF8574 pcf8574(PCF8574_address);



uint8_t h_mode = CLOCK_H12;

void getsec(char *psz)
{
  if(s>59){
    s=0;
  }
    sprintf(psz, "%02d", s);
}
void reDisplayP3(){
//pathi room 
       P_3.begin(3);
    P_3.setInvert(false);
    
    P_3.setZone(0, 0, 0);
    P_3.setZone(1, 1, 3);
    P_3.setFont(0, numeric7Seg);
    P_3.setFont(1, numeric7Se);
    P_3.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P_3.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

}

void reDisplayP0(){
  //preacher

       P_0.begin(3);
    P_0.setInvert(false);
    
    P_0.setZone(0, 0, 0);
    P_0.setZone(1, 1, 3);
    P_0.setFont(0, numeric7Seg);
    P_0.setFont(1, numeric7Se);
    P_0.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P_0.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

}
void reDisplayP2(){
  //pathi
       P_2.begin(3);
    P_2.setInvert(false);
    
    P_2.setZone(0, 0, 0);
    P_2.setZone(1, 1, 3);
    P_2.setFont(0, numeric7Seg);
    P_2.setFont(1, numeric7Se);
    P_2.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P_2.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

}

void reDisplayAllDisplays(){
     P_0.begin(3);
    P_1.begin(3);
    P_2.begin(3);
    P_3.begin(3);

// P_0.setIntensity(10);


    P_0.setInvert(false);
    P_1.setInvert(false);
    P_2.setInvert(false);
    P_3.setInvert(false);



    P_0.setZone(0, 0, 0);
    P_0.setZone(1, 1, 3);
    P_0.setFont(0, numeric7Seg);
    P_0.setFont(1, numeric7Se);
    P_0.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P_0.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

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
void setup(void)
{
  Wire.begin();

    Serial.begin(115200);
   // Sender.begin(115200, SERIAL_8N1, Sender_Txd_pin, Sender_Rxd_pin); // Define and start Sender serial port used for sending clock data 

    delay(10);
    //clock initialisation
    RTC.begin();

    //gpio expander initialisation 



   P_0.begin(3);
    P_1.begin(3);
    P_2.begin(3);
    P_3.begin(3);



    P_0.setInvert(false);
    P_1.setInvert(false);
    P_2.setInvert(false);
    P_3.setInvert(false);



    P_0.setZone(0, 0, 0);
    P_0.setZone(1, 1, 3);
    P_0.setFont(0, numeric7Seg);
    P_0.setFont(1, numeric7Se);
    P_0.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P_0.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

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

      //display P2
   pcf8574.pinMode(buttonPin1, INPUT);
    // pcf8574.pinMode(ledPin1, OUTPUT);

     //display P3
   //pcf8574.pinMode(buttonPin2, INPUT);
   //  pcf8574.pinMode(ledPin2, OUTPUT);

      //reDisplay button 
   pcf8574.pinMode(reDisplayButton, INPUT);
    

if (pcf8574.begin()){
    Serial.println("OK");
  }else{
    Serial.println("KO");
  }
    
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
int countOn=0;
int countOff=0;
void loop(void)
{


                  //P_0.displayClear();


    static uint32_t lastTime = 0;    // millis() memory
    static uint32_t lastTimeMin = 0; // millis() memory
    static uint32_t lastTimeHr = 0;  // millis() 
    static uint32_t lastTimeSend = 0;  // millis() memory
        static uint32_t lastTimeSendP0 = 0;  // millis() memory

    static uint32_t lastTimeSendP2 = 0;  // millis() memory



    static uint8_t display = 0;  // current display mode
    static bool flasher = false; // seconds passing flasher

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
         
          //gpio expander on off button handling
          int buttonState1= pcf8574.digitalRead(buttonPin1);
          Serial.println("hey lets check =========================================");
          Serial.print("       ");
          Serial.print(buttonState1);

          Serial.println("hey lets check =========================================");
          Serial.print("       ");
        //  Serial.print(pcf8574.digitalRead(buttonPin2));
                  //  delay(2000);

              if (buttonState1 == HIGH) {
                Serial.println("hey i am being the hight bioy button 1 turndedofff ------------------------------------- free --- ");
                
                  P_2.displayClear();
                   }
              else {
                Serial.println("hey i am ihte lo w freee ");
            
                   P_2.displayAnimate();
                  // pcf8574.digitalWrite(ledPin1, HIGH); // turn the LED on
                    }

                    
//                if (pcf8574.digitalRead(buttonPin2) == HIGH ) {
//                Serial.println("hey i am being the hight bioy button 2 iam turned off");
//                       P_3.displayClear();
//                   //  pcf8574.digitalWrite(ledPin2, LOW); // turn the LED on
//                   }
//              else {
//                Serial.println("hey i am ihte lo w");
//                     P_3.displayAnimate();
//                   //   pcf8574.digitalWrite(ledPin2, HIGH); // turn the LED on
//
//              }
        
                  //redisplay all 4 displays 
              if (pcf8574.digitalRead(reDisplayButton) == LOW) {
                reDisplayAllDisplays();
                Serial.println("Redisplay all 4 displays ");
                
                }

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

    //P_1.displayAnimate();

  //  P_2.displayAnimate();
    P_3.displayAnimate();

// sending data to slave 
  Serial.println(lastTime);

    if (millis() - lastTime >= 320 || i == 0)

    {  //gpio expander on off button handling
    //  reDisplayAllDisplays();
      
          int buttonState1= pcf8574.digitalRead(buttonPin1);
                    Serial.println("hey lets check ===========i==============================");
          Serial.print("       ");
          Serial.print(buttonState1);

          Serial.println("hey lets check =========================================");
          Serial.print("       ");
        //  Serial.print(pcf8574.digitalRead(buttonPin2));
          //delay(2000);
              if (buttonState1 == HIGH) {
                Serial.println("hey i am being the hight bioy button 1 turndedofff ------------------------------------- 200 milli ");
                
                  P_2.displayClear();
                   //pcf8574.digitalWrite(ledPin1, LOW); // turn the LED on
                   }
              else {
                Serial.println("hey i am ihte lo w -------------- button 1 milli  ");
            
                   P_2.displayAnimate();
                   //pcf8574.digitalWrite(ledPin1, HIGH); // turn the LED on
                           P_2.displayReset(0);
                           P_2.displayReset(1);                     


                    }
//                if (pcf8574.digitalRead(buttonPin2) == HIGH) {
//                Serial.println("hey i am being the hight bioy button 2 iam turned off");
//                       P_3.displayClear();
//                    // pcf8574.digitalWrite(ledPin2, LOW); // turn the LED on
//                   }
//              else {
//                Serial.println("hey i am ihte lo w");
//                     P_3.displayAnimate();
//                     // pcf8574.digitalWrite(ledPin2, HIGH); // turn the LED on
//                              P_3.displayReset(0);
//                              P_3.displayReset(1);
//              }

        lastTime = millis();
        // lastTimeMin=millis();
        getsec(szsecond);
        getTime(szTime, flasher);
                              P_3.displayReset(0);
                              P_3.displayReset(1);
                              
                                    P_0.displayAnimate();
                                    P_0.displayReset(0);
                                    P_0.displayReset(1);
                              
    P_1.displayAnimate();
        P_1.displayReset(0);
        P_1.displayReset(1);                     
    

    }
//
     if (millis() - lastTimeSend >= 50000 || i == 0)

    {  // pathi room 

        lastTimeSend = millis();
                                  // Send it to Sender serial port
            reDisplayP3();
    }
      if (millis() - lastTimeSendP2 >= 40000 || i == 0)

    {     
      
          // pathi
        lastTimeSendP2 = millis();
                                  // Send it to Sender serial port
            reDisplayP2();
    }
//      if (millis() - lastTimeSendP0 >= 30000 || i == 0)
//
//    {
//            //preacher
//        lastTimeSendP0 = millis();
//                                  // Send it to Sender serial port
//            reDisplayP0();
//    }




    i = 1;
        //delay(3000);

}
