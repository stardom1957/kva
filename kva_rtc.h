#ifndef _kva_rtc_h
#define _kva_rtc_h
#ifdef RTC_COMPILE

<<<<<<< HEAD
#define ARDUINO_SAM_DUE // for DUE RTC will run on SCL1 and SDA1
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and DUE Wire lib
=======
#include <Wire.h>
#include "RTClib.h" // Date and time functions using a DS3231 RTC connected via I2C and Wire lib
>>>>>>> c04aeab22ecbb02bb5ccc3397e782d76425e47b8

// RTC init global DS3231
RTC_DS3231 rtc;
DateTime now;

// retourne date et heure, avec ou sans secondes
String chaineDateHeure(bool secondes) {
  now = rtc.now();
  String dh = "";
  dh += now.year();
  //tft.print(now.year(), DEC);
  dh += "-";
  //tft.print('-');
  if (now.month() < 10) dh += "0";
  //if (now.month() < 10) tft.print('0');
  dh += now.month();
  dh += "-";
  if (now.day() < 10) dh += "0";
  dh += now.day();
  dh += " ";
  if (now.hour() < 10) dh += "0";
  dh += now.hour();
  dh += ":";
  if (now.minute() < 10) dh += "0";
  dh += now.minute();
  if (secondes){
    dh += ":";
    if (now.second() < 10) dh += "0";
    dh += now.second();
  }
  return dh;
}


void kva_rtc_init(void) {
// RTC init
  if (rtc.begin()) {
    rtcFound = true;
  }
  rtcNotInitialized = rtc.lostPower();
  if (rtcNotInitialized) {
    // set the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}
#endif //RTC_COMPILE
#endif
