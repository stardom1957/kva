#ifndef _kva_rtc_h
#define _kva_rtc_h
#include <Wire.h>
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and Wire lib

// RTC init global DS3231
RTC_DS3231 rtc;
DateTime now;

void kva_rtc_init(void) {
// RTC init
  if (rtc.begin()) {
    rtcFound = true;
  }

  if (rtc.lostPower()) {
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
}


#endif
