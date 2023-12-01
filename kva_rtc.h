#ifndef _kva_rtc_h
#define _kva_rtc_h
#ifdef RTC_COMPILE

#define ARDUINO_SAM_DUE // for DUE so that RTC will be connected to SCL1 and SDA1
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and DUE Wire lib

boolean rtcFound{false};      // RTC found or not

//RTC_DS3231 rtc;
RTC_DS1307 rtc;
DateTime now;
uint32_t daysInMonth[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

// returns date and time w or w/o seconds
String strDateTime(bool seconds) {
  now = rtc.now();
  //String dt{NULL};
  String dt{""};
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
boolean kva_rtc_init(void) {
  if (rtc.begin()) {
    debugln("RTC found!");
    digitalWrite(LED_YELLOW_ALERT_CONDITION, LOW);
    debugln(strDateTime(true));
    return true;
  }
  else {
    debugln("RTC not found!");
    Serial.flush();
    digitalWrite(LED_YELLOW_ALERT_CONDITION, HIGH);
    return false;
  }
}
#endif //RTC_COMPILE
#endif
