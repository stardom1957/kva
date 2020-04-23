#ifndef _kva_hmi_h
#define _kva_hmi_h

/**
 Notes:
 1.  modification to nexConfig.h see #define nextSerial ...
 
 */

#include "Nextion.h"
// pages create callback only if needed

//                                       (page, ID, objname)
NexText topmode  =                NexText(0, 4, "topmode");    // displays current opMode
NexProgressBar jbuffstat = NexProgressBar(0, 5, "jbuffstat");  // displays progress bar for message buffer status

// main menu buttons components repeat on each page
// note: all page change are handled by HMI, so no callback function required!
NexDSButton bstat1 =              NexDSButton(1, 7, "bstat1");       // page 1 button to status 0 CALLBACK REQUIRED
NexDSButton bstat2 =              NexDSButton(2, 1, "bstat2");       // page 2 button to status page 0 CALLBACK REQUIRED
NexDSButton bop0   =              NexDSButton(0, 7, "bop0");       // page 0 button to opMode selection page 1 CALLBACK REQUIRED
NexDSButton bop2   =              NexDSButton(2, 2, "bop2");       // page 2 button to opMode selection page 1 CALLBACK REQUIRED
NexDSButton brtc0  =              NexDSButton(0, 8, "brtc0");       // page 0 button to rtc set page 2 CALLBACK REQUIRED
NexDSButton brtc1  =              NexDSButton(1, 9, "brtc1");       // page 1 button to rtc set page 2 CALLBACK REQUIRED

// page 1 set opMode components
NexDSButton bsby      = NexDSButton(1, 2, "bsby");        // STANDBY mode select button CALLBACK REQUIRED
NexDSButton btele     = NexDSButton(1, 4, "btele");       // TELEOPY mode select button CALLBACK REQUIRED
NexDSButton bfrun     = NexDSButton(1, 3, "bfrun");       // FREE_RUN mode select button CALLBACK REQUIRED
NexNumber ngotimer      = NexNumber(1, 6, "ngotimer");    // time count for timer to start selected opMode

// page 2 set RTC date and time components
NexNumber nyear =      NexNumber(2, 4, "nyear");    // number field for year >= 2020
NexNumber nmonth =     NexNumber(2, 5, "nmonth");   // number field for month
NexNumber nday =       NexNumber(2, 6, "nday");     // number field for day
NexNumber nhour =      NexNumber(2, 7, "nhour");    // number field for hours
NexNumber nmin =       NexNumber(2, 8, "nmin");     // number field fo minutes
NexNumber nsec =       NexNumber(2, 9, "nsec");     // number field for seconds
NexDSButton btdate = NexDSButton(2, 16, "btdate");  // double state button to select date line for modif
NexDSButton bttime = NexDSButton(2, 17, "bttime");  // double state button to select time line for modif
NexButton bplus =      NexButton(2, 18, "bplus");   // button to add 5 to current fiels being modified CALLBACK REQUIRED
NexButton bminus =     NexButton(2, 19, "bminus");  // button to substract 1 from current field being modified CALLBACK REQUIRED
NexDSButton btyh =   NexDSButton(2, 13, "btyh");    // double state button to select year and hour row for modif
NexDSButton btmm =   NexDSButton(2, 14, "btmm");    // double state button to select month and minutes row for modif
NexDSButton btds =   NexDSButton(2, 15, "btds");    // double state button to select day and seconds row for modif
NexButton bsetRTC =    NexButton(2, 21, "bsetRTC"); // button to set RTC CALLBACK REQUIRED

// register components that have a CALLBACK
NexTouch *nex_listen_list[] = 
{
 &bop0, &bop2, &bsby, &btele, &bfrun, &bstat1, &bstat2,
 &brtc0, &brtc1, &bplus, &bminus, &bsetRTC, NULL
};

/* ABOVE, removed unnecessary components 
NexTouch *nex_listen_list[] = 
{
 &bop0, &bop2, &bsby, &btele, &bfrun, &bstat1, &bstat2,
 &brtc0, &brtc1, &ngotimer, &nyear, &nmonth,
 &nday, &nhour, &nmin,  &nsec,  &btdate,   &bttime, &bplus,
 &bminus, &btyh, &btmm, &btds, &bsetRTC, NULL
};

*/

char char_buffer[25] = {0};              // C-style char buffer to hold String data
unsigned long timerOm{0};                // timer count to control delay to start selected opMode via HMI
unsigned long displayTimer{0};           // timer count to control dislay of status to HMI
unsigned long displayInterval{1000};     // display status interval in ms
uint32_t delayStatOpMode = (uint32_t)10; // delay in s
uint32_t ngotimer_val = delayStatOpMode; // delay to start opMode
byte currentHMIpage{0};                  // the page the HMi is currently showing, page 0 at start

//***************************************
//***************************************

// page change callback in order to get currentHMIpage
// change to page 0
void bstat1PopCallback(void *ptr) {
  dbSerialPrintln("bstat1PopCallback");
  currentHMIpage = 0;
}

void bstat2PopCallback(void *ptr) {
  dbSerialPrintln("bstat2PopCallback");
  currentHMIpage = 0;
}

// change to page 1
void bop0PopCallback(void *ptr) {
  dbSerialPrintln("bop0PopCallback");
  currentHMIpage = 1;
}

void bop2PopCallback(void *ptr) {
  dbSerialPrintln("bop2PopCallback");
  currentHMIpage = 1;
}

// change to page 2
void brtc0PopCallback(void *ptr) {
  dbSerialPrintln("brtc0PopCallback");
  currentHMIpage = 2;
}

void brtc1PopCallback(void *ptr) {
  dbSerialPrintln("brtc1PopCallback");
  currentHMIpage = 2;
}

// add +5 to currently RTC selected field on page 2
void bplusPopCallback(void *ptr) {
  dbSerialPrintln("bplusPopCallback");
  // TODO:
  //  find currently selected field
  //  ADD 5 to it, check and ajust to limits
}

// subtract 1 from currently RTC selected field on page 2
void bminusPopCallback(void *ptr) {
  dbSerialPrintln("bminusPopCallback");
  // TODO:
  //  find currently selected field
  //  SUBSTRACT 1 from it, check and ajust to limits
}

void bsetRTCPopCallback(void *ptr) {
  dbSerialPrintln("bsetRTCPopCallback");
  // #TODO:
  // check of all fields have valid values
  // set RTC
  // check validity of set
  // feed back (highlight proprer fields) in case of incorrect value
}

// call back of bsby select STANDBY opMode
void bsbyPopCallback(void *ptr)
{
  // reset delay timer
  dbSerialPrintln("bsbyPopCallback");
  dbSerialPrintln("  ");
  //ngotimer.setValue(delayStatOpMode);
  //timerOm = millis(); // start timer
  selectedOpMode = STANDBY; //set opMode desired
  opModeChangeRequested = true;
}

// call back of btele select TELEOP opMode
void btelePopCallback(void *ptr)
{
  // reset delay timer
  dbSerialPrintln("btelePopCallback");
  dbSerialPrintln("  ");
  //ngotimer.setValue(delayStatOpMode);
  //timerOm = millis(); // start timer
  selectedOpMode = TELEOP; //set opMode desired
  opModeChangeRequested = true;
}

// call back of bfrun select FREE_RUN opMode
void bfrunPopCallback(void *ptr)
{
  // reset delay timer
  dbSerialPrintln("bfrunPopCallback");
  dbSerialPrintln("  ");
  //ngotimer.setValue(delayStatOpMode);
  //timerOm = millis(); // start timer
  selectedOpMode = FREE_RUN; //set opMode desired
  opModeChangeRequested = true;
}

// manages change of opMode at the end of timer
void manageOpModeTimer(void) {
  //uint32_t oldTimer;
  dbSerialPrintln("manageOpModeTimer");
  dbSerialPrintln("  ");
  //dbSerialPrintln("...timer=");
  //dbSerialPrintln(oldTimer);

  //update display on HMI every second
  //ngotimer.getValue(&oldTimer);
/*  if (oldTimer != 0) {
    if (millis() > timerOm + 1000) {
      //ngotimer.getValue(&oldTimer);
      oldTimer-=1;
      ngotimer.setValue(oldTimer);
      timerOm = millis();
    }
  }
  else {
    standby(); // first go in STANDBY mode to reset every thing
    currentOpMode = selectedOpMode;
    opModeChangeRequested = false;
    //debug
    Serial.print("selectedOpMode: ");
    Serial.println(selectedOpMode);
  }
*/
    Serial.print("selectedOpMode: ");
    Serial.println(selectedOpMode);
    opModeChangeRequested = false;

}

//place into loop()
//    nexLoop(nex_listen_list);


#endif
