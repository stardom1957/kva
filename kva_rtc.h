#ifndef _kva_rtc_h
#define _kva_rtc_h
#ifdef RTC_COMPILE

#define ARDUINO_SAM_DUE // for DUE RTC that will run on SCK1 and SDA1
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and DUE Wire lib

boolean rtcFound{false};      // RTC found or not
boolean rtcInitialized{true}; // found RTC, but not initialized for ex. due to power lost

RTC_DS3231 rtc;
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
    // if year==02165, the RTC cannot be read
    // if year==2000, the RTC has probably lost power
    now = rtc.now();
    if (now.year() == 2000 || now.year() == 02165) rtcInitialized = false;
    //debug("debug year= ");
    //Serial.println(now.year());
    
    if (!rtcInitialized) {
      // set the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      debug("RTC set to compile time: ");
      debug(F(__DATE__));
      debug(" ");
      Serial.println(F(__TIME__));
      rtcInitialized = true;
    }
  }
  //debug Serial.println(strDateTime(true));
}
#endif //RTC_COMPILE
#endif
