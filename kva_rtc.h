#ifndef _kva_rtc_h
#define _kva_rtc_h
#ifdef RTC_COMPILE

#define ARDUINO_SAM_DUE // for DUE RTC that will run on SCK1 and SDA1
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and DUE Wire lib

boolean rtcFound{false};     // RTC found or not
boolean rtcNotInitialized{false}; // found RTC, but not initialized (probably power lost condition)

RTC_DS3231 rtc;
DateTime now;
uint32_t daysInMonth[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// returns date and time w or w/o seconds
String strDateTime(bool secondes) {
  now = rtc.now();
  String dt{NULL};
  dt += now.year();
  //tft.print(now.year(), DEC);
  dt += "-";
  //tft.print('-');
  if (now.month() < 10) dt += "0";
  //if (now.month() < 10) tft.print('0');
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
  if (secondes){
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
    rtcNotInitialized = rtc.lostPower();
    if (rtcNotInitialized) {
      // set the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      rtcNotInitialized = false;
    }
  }
}
#endif //RTC_COMPILE
#endif
