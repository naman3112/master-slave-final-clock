// Header file includes
#include <WiFi.h>
#include <time.h>

#include <MD_Parola.h>
#include <SPI.h>
#include <Font_Data.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4

#define CLK_PIN1 18
#define DATA_PIN1 19
#define CS_PIN1 5

unsigned long debounceDelay = 50;

#include <DS1307.h>
DS1307 rtc;

MD_Parola P_1 = MD_Parola(HARDWARE_TYPE, DATA_PIN1, CLK_PIN1, CS_PIN1, MAX_DEVICES);

#define SPEED_TIME 75
#define PAUSE_TIME 0
#define MAX_MESG 20

#define BUTTON_SEC 13
#define BUTTON_PIN 15
#define BUTTON_MIN 4

#include <Wire.h>

const int buttonPin1 = 0;
const int reDisplayButton = 1;

int h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 1, s2 = 1, hours = 01, minutes = 01, seconds = 0;
int lastState = HIGH, lastMin = HIGH, lastSec = HIGH;
int currentState, currentMin, currentSec;

const int timezoneinSeconds = 19800;
int dst = 0;
uint16_t h = 01, m = 01, s = 12;

char szTime[9];
char szsecond[4];
char szMesg[MAX_MESG + 1] = "";
uint8_t sec, minute, hour, day, month;
uint16_t year;

void getTime(char *psz, bool f = true) {
  sprintf(psz, "%02d%c%02d%c", h, (f ? ':' : ':'), m, ':');
  if (P_1.getZoneStatus(1)) P_1.displayReset(1);
  Serial.println(psz);
}

void getsec(char *psz) {
  if (s > 59) s = 0;
  if (P_1.getZoneStatus(0)) {
    sprintf(psz, "%02d", s);
    P_1.displayReset(0);
  }
}

void setup(void) {
  rtc.begin();
  rtc.start();
  rtc.get(&sec, &minute, &hour, &day, &month, &year);

  Serial.begin(9600);
  Wire.begin();

  P_1.begin(3);
  P_1.setInvert(false);
  P_1.setZone(0, 0, 0);
  P_1.setZone(1, 1, 3);
  P_1.setFont(0, numeric7Seg);
  P_1.setFont(1, numeric7Se);
  P_1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MIN, INPUT_PULLUP);
  pinMode(BUTTON_SEC, INPUT_PULLUP);

  getTime(szTime);
}

int i = 0;

int ConvertIntoSeconds(int h, int m, int s) {
  return h * 3600 + m * 60 + s;
}

int countOn = 0, countOff = 0;

void loop(void) {
  rtc.get(&sec, &minute, &hour, &day, &month, &year);
  h = hour;
  m = minute;
  s = sec;

  static uint32_t lastTime = 0, lastTimeMin = 0, lastTimeHr = 0, lastTimeSendData = 0;
  static bool flasher = false;

  currentState = digitalRead(BUTTON_PIN);
  currentMin = digitalRead(BUTTON_MIN);
  currentSec = digitalRead(BUTTON_SEC);

  if (lastState == LOW && currentState == HIGH && millis() - lastTimeHr >= 400) {
    lastTimeHr = millis();
    h++;
    if (h > 12) h = 1;
    rtc.set(s, m, h, day, month, year);
  }

  if (lastMin == LOW && currentMin == HIGH && millis() - lastTimeMin >= 400) {
    lastTimeMin = millis();
    m++;
    if (m > 59) m = 0;
    rtc.set(s, m, h, day, month, year);
  }

  if (lastSec == LOW && currentSec == HIGH) {
    s = 0;
    rtc.set(s, m, h, day, month, year);
  }

  lastState = currentState;
  lastMin = currentMin;
  lastSec = currentSec;

  if (millis() - lastTime >= 100 || i == 0) {
    P_1.displayAnimate();
    lastTime = millis();
    getsec(szsecond);
    getTime(szTime, flasher);
  }

  if (millis() - lastTimeSendData >= 3000 || i == 0) {
    lastTimeSendData = millis();
  }

  i = 1;
}
