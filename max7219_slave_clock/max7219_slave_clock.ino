// Header file includes
#include <WiFi.h>
#include <time.h>
#include <MD_Parola.h>
#include <SPI.h>

#include "Font_Data.h"

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4

#define CLK_PIN1 18  // or SCK
#define DATA_PIN1 19 // or MOSI
#define CS_PIN1 5    // or SS

// Part - 2 check
#define CLK_PIN 14  // or SCK
#define DATA_PIN 12 // or MOSI
#define CS_PIN 2    // or SS

// Arbitrary output pins
MD_Parola P = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
MD_Parola P1 = MD_Parola(HARDWARE_TYPE, DATA_PIN1, CLK_PIN1, CS_PIN1, MAX_DEVICES);

#define SPEED_TIME 75
#define PAUSE_TIME 0
#define MAX_MESG 20

#define BUTTON_1 13 // Display 1 on/off
#define BUTTON_2 15 // Display 2 on/off

//led indicators
 
#define LED_1 32 // Display 1 on/off
#define LED_2 33 // Display 2 on/


// RTC
#include <Wire.h>

// SCL D22
// SDA D21

int h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 1, s2 = 1, hours = 12, minutes = 12, seconds = 0;

// Sending/Receiving example

HardwareSerial Receiver(2); // Define a Serial port instance called 'Receiver' using serial port 2

#define Receiver_Txd_pin 17
#define Receiver_Rxd_pin 16


/**********  User Config Setting   ******************************/

// calculate your timezone in seconds,1 hour = 3600 seconds and 5.30Hrs = 19800
const int timezoneinSeconds = 19800;
/***************************************************************/
int dst = 0;
uint16_t h=12, m=12, s=12;

// Global variables
char szTime[9];   // mm:ss\0
char szsecond[4]; // ss
char szMesg[MAX_MESG + 1] = "";


void getsec(char *psz)
{
  if(s>59){
    s=0;
  }
    sprintf(psz, "%02d", s);
}

void getTime(char *psz, bool f = true)
{

   
    Serial.print(h);
 
   
    sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
    Serial.println(psz);
    Serial.println("Hey in the loop dude");
    Serial.println(h);
    Serial.println(m);
    Serial.println(s);
}

void setup(void)

{   
      Serial.begin(115200);

  Receiver.begin(115200, SERIAL_8N1, Receiver_Txd_pin, Receiver_Rxd_pin); // Define and start Receiver serial port


    P.begin(3);
    P1.begin(3);
    P.setInvert(false);
    P1.setInvert(false);

    P.setZone(0, 0, 0);
    P.setZone(1, 1, 3);
    P.setFont(0, numeric7Seg); //seconds fonts
    P.setFont(1, numeric7Se);//minutes fonts
    P.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

    // pathi display
    P1.setZone(0, 0, 0);
    P1.setZone(1, 1, 3);
    P1.setFont(0, numeric7Seg);//seconds 
    P1.setFont(1, numeric7Se);//min 
    P1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
    P1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

      pinMode(BUTTON_1, INPUT_PULLUP);
      pinMode(BUTTON_2, INPUT_PULLUP);

      //LED
      pinMode(LED_1, OUTPUT);
      pinMode(LED_2, OUTPUT);




    //  delay(100);

    getTime(szTime);

}
int i = 0;

void buildTime(int  recievedSeconds){
 


  
  m = recievedSeconds / 60;
s = recievedSeconds % 60;
h = m / 60;
m = m % 60;
  
}
int recieved_seconds;
int prevData=0;
int count =0;
void loop()
{        Serial.println("hey i am in loop");
//      while(Receiver.available()){
//                Serial.println("hey i am in reciecver");
//                //Serial.println()
//
//
//         int recieved_seconds = Receiver.parseInt();// Receiver.parseInt(); // Display the Receivers characters
//              
//            Serial.println("recieved_seconds");
//            Serial.println(recieved_seconds);
//            buildTime(recieved_seconds);
//
//            Serial.print("hours");
//            Serial.println(h);
//
//            
//            Serial.print("minutes");
//            Serial.println(m);
//
//            
//            Serial.print("seconss");
//            Serial.println(s);
//            
// 
//      }
      
      
             static uint32_t lastTime = 0;    // millis() memory
             static uint32_t lastTimeRecieve = 0;    // millis() memory
static uint32_t lastTimePlus=0;
             static uint32_t lastTimeMin = 0; // millis() memory
             static uint32_t lastTimeHr = 0;  // millis() memory
             static uint8_t display = 0;  // current display mode
             static bool flasher = false; // seconds passing flasher
    
              if (digitalRead(BUTTON_1) == HIGH) {
                Serial.println("hey i am being the hight bioy");
                   //    P.displayClear();
                  P1.displayClear();
                   digitalWrite(LED_1, LOW); // turn the LED on
                   }
              else {
                Serial.println("hey i am ihte lo w");
            //  P.displayAnimate();
                   P1.displayAnimate();
                   digitalWrite(LED_1, HIGH); // turn the LED on
                    }
                if (digitalRead(BUTTON_2) == HIGH) {
                Serial.println("hey i am being the hight bioy");
                       P.displayClear();
                     digitalWrite(LED_2, LOW); // turn the LED on
                   }
              else {
                Serial.println("hey i am ihte lo w");
                     P.displayAnimate();
                      digitalWrite(LED_2, HIGH); // turn the LED on

                    }

                    


            if (millis() - lastTimeRecieve >= 1000 )

         {
               lastTimeRecieve = millis();
               
                while(Receiver.available()){
                Serial.println("hey i am in reciecver");
                //Serial.println()


         //int recieved_seconds = Receiver.parseInt();// Receiver.parseInt(); // Display the Receivers characters
             
              int currData = Receiver.parseInt();
              if(currData!=prevData){
                  recieved_seconds = currData;
                  recieved_seconds++;
                 buildTime(recieved_seconds);
                    count=0;
              }else{
                count++;
              }
              prevData=currData;
            Serial.println("recieved_seconds");
            Serial.println(recieved_seconds);
            //buildTime(recieved_seconds);

            Serial.print("hours");
            Serial.println(h);

            
            Serial.print("minutes");
            Serial.println(m);

            
            Serial.print("seconss");
            Serial.println(s);

 
          }

                       
      }
     if (millis() - lastTime >= 100 || i == 0)

         {
                              //buildTime(recieved_seconds);
      

               lastTime = millis();
                getsec(szsecond);
               getTime(szTime, flasher);
   if (digitalRead(BUTTON_1) == HIGH) {
                Serial.println("hey i am being the hight bioy");
                       //P.displayClear();
                      P1.displayClear();
                       digitalWrite(LED_1, LOW); // turn the LED on

                   }
              else {
                Serial.println("hey i am ihte lo w");
                // P.displayReset(0);
               P1.displayReset(0);
               // P.displayReset(1);
              P1.displayReset(1);
                    digitalWrite(LED_1, HIGH); // turn the LED on


                    }

                      if (digitalRead(BUTTON_2) == HIGH) {
                Serial.println("hey i am being the hight bioy");
                       P.displayClear(); 
                       digitalWrite(LED_2, LOW); // turn the LED on                      
                   }
              else {
                Serial.println("hey i am ihte lo w");
                P.displayReset(0);

                P.displayReset(1);
                digitalWrite(LED_2, HIGH); // turn the LED on


                    }
           
                       
      }


  if (millis() - lastTimePlus >= 900 || i == 0)

         {
                        
                          recieved_seconds++;
                       
                
                            buildTime(recieved_seconds);

//                  getsec(szsecond);
//               getTime(szTime, flasher);
//
//               P.displayReset(0);
//               P1.displayReset(0);
//                P.displayReset(1);
//               P1.displayReset(1);
               lastTimePlus = millis();
                       
      }
      
        i = 1;
        
//delay(2000);

}
