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

#define DATA_PIN2 32
#define CLK_PIN2 33
#define CS_PIN2 25

#define CLK_PIN3 26
#define DATA_PIN3 23
#define CS_PIN3 27

#include <RTClib.h>
RTC_DS3231 rtc;

#include <Wire.h>

MD_Parola P_1 = MD_Parola(HARDWARE_TYPE, DATA_PIN1, CLK_PIN1, CS_PIN1, MAX_DEVICES);
MD_Parola P_2 = MD_Parola(HARDWARE_TYPE, DATA_PIN2, CLK_PIN2, CS_PIN2, MAX_DEVICES);
MD_Parola P_3 = MD_Parola(HARDWARE_TYPE, DATA_PIN3, CLK_PIN3, CS_PIN3, MAX_DEVICES);

#define SPEED_TIME 75
#define PAUSE_TIME 0

#define BUTTON_SEC 13
#define BUTTON_PIN 15
#define BUTTON_MIN 4

int lastState = HIGH, lastMin = HIGH, lastSec = HIGH;
int currentState, currentMin, currentSec;

uint16_t h = 1, m = 1, s = 0;

char szTime[9];
char szsecond[4];

uint8_t sec, minute, hour, day, month;
uint16_t year;

// 🔥 Convert 24h → 12h
int to12Hour(int hh) {
  if (hh == 0) return 12;
  if (hh > 12) return hh - 12;
  return hh;
}

// 🔥 Disable DS3231 alarms
void disableDS3231Alarms() {
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);

  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  rtc.writeSqwPinMode(DS3231_OFF);

  Wire.beginTransmission(0x68);
  Wire.write(0x0E);
  Wire.write(0b00000100);
  Wire.endTransmission();
}

void getTime(char *psz) {
  sprintf(psz, "%02d:%02d:", h, m);

  if (P_1.getZoneStatus(1)) P_1.displayReset(1);
  if (P_2.getZoneStatus(1)) P_2.displayReset(1);
  if (P_3.getZoneStatus(1)) P_3.displayReset(1);
}

void getsec(char *psz) {
  if (s > 59) s = 0;

  sprintf(psz, "%02d", s);

  if (P_1.getZoneStatus(0)) P_1.displayReset(0);
  if (P_2.getZoneStatus(0)) P_2.displayReset(0);
  if (P_3.getZoneStatus(0)) P_3.displayReset(0);
}

void setup(void) {
  Wire.begin();

  if (!rtc.begin()) {
    while (1);
  }

  disableDS3231Alarms();

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // run once only

  P_1.begin(3);
  P_2.begin(3);
  P_3.begin(3);

  P_1.setInvert(false);
  P_2.setInvert(false);
  P_3.setInvert(false);

  // P1
  P_1.setZone(0, 0, 0);
  P_1.setZone(1, 1, 3);
  P_1.setFont(0, numeric7Seg);
  P_1.setFont(1, numeric7Se);
  P_1.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_1.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

  // P2
  P_2.setZone(0, 0, 0);
  P_2.setZone(1, 1, 3);
  P_2.setFont(0, numeric7Seg);
  P_2.setFont(1, numeric7Se);
  P_2.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_2.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

  // P3
  P_3.setZone(0, 0, 0);
  P_3.setZone(1, 1, 3);
  P_3.setFont(0, numeric7Seg);
  P_3.setFont(1, numeric7Se);
  P_3.displayZoneText(0, szsecond, PA_LEFT, SPEED_TIME, 0, PA_PRINT, PA_NO_EFFECT);
  P_3.displayZoneText(1, szTime, PA_LEFT, SPEED_TIME, PAUSE_TIME, PA_PRINT, PA_NO_EFFECT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MIN, INPUT_PULLUP);
  pinMode(BUTTON_SEC, INPUT_PULLUP);
}

int i = 0;

void loop(void) {
  DateTime now = rtc.now();

  sec = now.second();
  minute = now.minute();
  hour = now.hour();
  day = now.day();
  month = now.month();
  year = now.year();

  // 🔥 Display in 12-hour
  h = to12Hour(hour);
  m = minute;
  s = sec;

  static uint32_t lastTime = 0, lastTimeMin = 0, lastTimeHr = 0;

  currentState = digitalRead(BUTTON_PIN);
  currentMin = digitalRead(BUTTON_MIN);
  currentSec = digitalRead(BUTTON_SEC);

  // 🔥 Hour button (12-hour cycle but stored in 24h)
  if (lastState == LOW && currentState == HIGH && millis() - lastTimeHr >= 400) {
    lastTimeHr = millis();

    hour++;
    if (hour >= 24) hour = 0;

    rtc.adjust(DateTime(year, month, day, hour, minute, sec));
  }

  // Minute button
  if (lastMin == LOW && currentMin == HIGH && millis() - lastTimeMin >= 400) {
    lastTimeMin = millis();

    minute++;
    if (minute >= 60) minute = 0;

    rtc.adjust(DateTime(year, month, day, hour, minute, sec));
  }

  // Second reset
  if (lastSec == LOW && currentSec == HIGH) {
    sec = 0;
    rtc.adjust(DateTime(year, month, day, hour, minute, sec));
  }

  lastState = currentState;
  lastMin = currentMin;
  lastSec = currentSec;

  if (millis() - lastTime >= 100 || i == 0) {
    P_1.displayAnimate();
    P_2.displayAnimate();
    P_3.displayAnimate();

    lastTime = millis();
    getsec(szsecond);
    getTime(szTime);
  }

  i = 1;
}