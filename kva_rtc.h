#ifndef _kva_rtc_h
#define _kva_rtc_h
#ifdef RTC_COMPILE

#define ARDUINO_SAM_DUE // for DUE so that RTC will be connected to SCL1 and SDA1
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and DUE Wire lib

boolean rtcFound{false};      // RTC found or not
boolean rtcInitialized{true}; // found RTC, but not initialized for ex. due to power lost

//RTC_DS3231 rtc;
RTC_DS1307 rtc;
DateTime now;
uint32_t daysInMonth[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// returns date and time w or w/o seconds
String strDateTime(bool seconds) {
  now = rtc.now();
  String dt{NULL};
  dt = now.year();
  dt += "-";
  if (now.month() < 10) dt += "0";
  dt += now.month();
  dt += "-";
  if (now.day() < 10) dt += "0";
  dt += now.day();
  dt += " ";
  if (now.hour() < 10) dt += "0";
  dt += now.hour();
  dt += ":";
  if (now.minute() < 10) dt += "0";
  dt += now.minute();
  if (seconds){
    dt += ":";
    if (now.second() < 10) dt += "0";
    dt += now.second();
  }
  return dt;
}

// RTC init
void kva_rtc_init(void) {
  if (rtc.begin()) {
    rtcFound = true;
    debugln("RTC found!");
    digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW);
    rtcInitialized = true;
    /*
    if (rtc.lostPower()) {
      // set the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      debug("RTC set to compile time: ");
      debug(F(__DATE__));
      debug(" ");
      debugln(F(__TIME__));
      rtcInitialized = true;
    }
    */
    debugln(strDateTime(true));
  }
  else {
    debugln("RTC not found!");
    rtcInitialized = false;
    Serial.flush();
    digitalWrite(LED_YELLOW_ALERT_CONDITION, HIGH);
    rtcFound = false;
  }
}
#endif //RTC_COMPILE
#endif
