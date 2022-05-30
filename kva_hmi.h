#ifndef _kva_hmi_h
#define _kva_hmi_h

/**
 Notes:
 1. modified nexConfig.h: #define nexSerial Serial2 for Serial2 on Due
 2. for debugging, nexConfig.h: uncomment #define DEBUG_SERIAL_ENABLE
 
 */

#include "Nextion.h"

void strToChar(String);

// put part of a string into char char_buffer
// needed for HMI text handling
void strToChar(String s) {
  memset(char_buffer, 0, sizeof(char_buffer));
  for (byte i=0; i < s.length(); i++) {
    char_buffer[i] = s.charAt(i);
  }
}

//define some constants for HMI color specific for Nextion
#define GREEN 1024
#define GREY 50712
#define YELLOW 65504
#define RED 63488

// hmi STATUS
boolean hmiFound{false};

// page components
NexPage page0    = NexPage(0, 0, "page0"); // main status page and default page
NexPage page1    = NexPage(1, 0, "page1"); // set opMode page
NexPage page2    = NexPage(2, 0, "page2"); // set RTC page

//                                       (page, ID, objname)
NexText topmode  =                NexText(0, 1, "topmode");    // displays current opMode
NexText tready   =                NexText(0, 5, "tready");     // vehicule ready indicator
NexText trtc     =                NexText(0, 6, "trtc");       // RTC ready indicator
NexText ttime    =                NexText(0, 7, "ttime");       // time field

NexProgressBar jbuffstat = NexProgressBar(0, 2, "jbuffstat");  // displays progress bar for message buffer status

// menu buttons components repeat on each page
// note: all page change are handled by HMI, so no callback function required!
// Nextion Editor kva_menu_7.hmi
NexDSButton bstat1 =              NexDSButton(1, 4, "bstat1"); // page 1 button to status 0 CALLBACK REQUIRED
NexDSButton bstat2 =              NexDSButton(2, 1, "bstat2"); // page 2 button to status page 0 CALLBACK REQUIRED
NexDSButton bop0   =              NexDSButton(0, 3, "bop0");   // page 0 button to opMode selection page 1 CALLBACK REQUIRED
NexDSButton bop2   =              NexDSButton(2, 2, "bop2");   // page 2 button to opMode selection page 1 CALLBACK REQUIRED
NexDSButton brtc0  =              NexDSButton(0, 4, "brtc0");  // page 0 button to rtc set page 2 CALLBACK REQUIRED
NexDSButton brtc1  =              NexDSButton(1, 5, "brtc1");  // page 1 button to rtc set page 2 CALLBACK REQUIRED

// page 1 set opMode components
NexButton bsby      = NexButton(1, 6, "bsby");        // STANDBY mode select button CALLBACK REQUIRED
NexButton bfrun     = NexButton(1, 7, "bfrun");       // FREE_RUN mode select button CALLBACK REQUIRED
NexButton btele     = NexButton(1, 8, "btele");       // TELEOPY mode select button CALLBACK REQUIRED
NexNumber ngotimer  = NexNumber(1, 3, "ngotimer");    // time count for timer to start selected opMode

// page 2 set RTC date and time components
NexNumber nyear =      NexNumber(2, 3, "nyear");    // number field for year >= 2020
NexNumber nmonth =     NexNumber(2, 4, "nmonth");   // number field for month
NexNumber nday =       NexNumber(2, 5, "nday");     // number field for day
NexNumber nhour =      NexNumber(2, 6, "nhour");    // number field for hours
NexNumber nmin =       NexNumber(2, 7, "nmin");     // number field fo minutes
NexNumber nsec =       NexNumber(2, 8, "nsec");     // number field for seconds
NexDSButton btdate = NexDSButton(2, 15, "btdate");  // double state button to select date line for modif
NexDSButton bttime = NexDSButton(2, 16, "bttime");  // double state button to select time line for modif
NexButton bplus5 =      NexButton(2, 17, "bplus5");   // button to add 5 to current fiels being modified CALLBACK REQUIRED
NexButton bplus1 =      NexButton(2, 21, "bplus5");   // button to add 5 to current fiels being modified CALLBACK REQUIRED
NexButton bminus =     NexButton(2, 18, "bminus");  // button to substract 1 from current field being modified CALLBACK REQUIRED
NexDSButton btyh =   NexDSButton(2, 12, "btyh");    // double state button to select year and hour row for modif
NexDSButton btmm =   NexDSButton(2, 13, "btmm");    // double state button to select month and minutes row for modif
NexDSButton btds =   NexDSButton(2, 14, "btds");    // double state button to select day and seconds row for modif
NexButton bsetRTC =    NexButton(2, 20, "bsetRTC"); // button to set RTC CALLBACK REQUIRED

// register components that have a CALLBACK
NexTouch *nex_listen_list[] = 
{
 &bop0, &bop2, &bsby, &btele, &bfrun, &bstat1, &bstat2,
 &brtc0, &brtc1, &bplus5, &bplus1, &bminus, &bsetRTC, 
 &page0, &page1, &page2, NULL
};

/* ABOVE, removed unnecessary components from this list
NexTouch *nex_listen_list[] = 
{
 &bop0, &bop2, &bsby, &btele, &bfrun, &bstat1, &bstat2,
 &brtc0, &brtc1, &ngotimer, &nyear, &nmonth,
 &nday, &nhour, &nmin,  &nsec,  &btdate,   &bttime, &bplus5,
 &bminus, &btyh, &btmm, &btds, &bsetRTC, NULL
};

*/

unsigned long displayTimer{0};           // timer count to control dislay of status to HMI
unsigned long displayInterval{1000};     // display status interval in ms
uint32_t delayStatOpMode = (uint32_t)10; // delay in s
uint32_t ngotimer_val = delayStatOpMode; // delay to start opMode
byte currentHMIpage{0};                  // the page the HMi is currently showing, page 0 at start

//***************************************
//***************************************
// page changes callbacks

// on page 0, button brtc0 pressed, page 2 requested
// page 2 is RTC setup
void brtc0PopCallback(void *ptr) {
  currentHMIpage = 2;
  // if rtc is ok send actual date and time values to HMI number fields
  if (rtcFound) {updateDTtoHMI();}
}

// on page 0, button bop0 pressed, page 1 requested
// page 1 is opmode change
void bop0PopCallback(void *ptr) {
  dbSerialPrintln("bop0PopCallback");
  currentHMIpage = 1;
  setOpmodeButtonColors(); // set opmode btn color according to current opmode
}

// on page 1, button bstat1 pressed, page 0 requested
// page 0 is status
void bstat1PopCallback(void *ptr) {
  dbSerialPrintln("bstat1PopCallback");
  currentHMIpage = 0;
}

// on page 2 button brtc1 pressed, page 2 requested
// page 2 is RTC setup
void brtc1PopCallback(void *ptr) {
  dbSerialPrintln("brtc1PopCallback");
  currentHMIpage = 2;
  // if rtc is ok send actual date and time values to HMI number fields
  if (rtcFound) updateDTtoHMI();
}


// on page 2, button bstat2 pressed, page 0 requested
// page 0 is status
void bstat2PopCallback(void *ptr) {
  dbSerialPrintln("bstat2PopCallback");
  currentHMIpage = 0;
}

// on page 2, button bop2 pressed, page 1 requested
// page 1 is opmode change
void bop2PopCallback(void *ptr) {
  dbSerialPrintln("bop2PopCallback");
  currentHMIpage = 1;
  setOpmodeButtonColors(); // set opmode btn color according to current opmode
}

/*
 * this will ajust the color of each mode select buttons
 * according to the situation of the mode selection variables
 * IMPORTANT NOTE: on the HMI, upon entering the page for opmode selection, all btn are set to GREY
 * so that we only have to do 2 passes
*/
 
void setOpmodeButtonColors(void) {
  // first pass: set color of button of current opMode to green
  switch(currentOpMode) {
    debugln("in setOpmodeButtonColors, first pass");
    //**********************************************
    //***************** STANDBY *******************
    case STANDBY:
     bsby.Set_background_color_bco(GREEN);
     break;

    case SENSORS_DEVELOPEMENT:
     break;

    case RUN_PRESET_COURSE:
     break;

    //**********************************************
    //***************** TELEOP *******************
    case TELEOP: // teleoperation by remote
     btele.Set_background_color_bco(GREEN);
     break;

    //**********************************************
    //***************** FREE_RUN *******************
    case FREE_RUN:
     bfrun.Set_background_color_bco(GREEN);
     break;

    //************************************************
    //******** MEASURE_AND_CALIBRATE_MOTORS **********
    case MEASURE_AND_CALIBRATE_MOTORS:
     break;
    
    default:
     ;
  } // end switch(currentOpMode)

  // if opModeChangeRequested do a second pass to set color
  // of button of requested opMode to yellow
  if (opModeChangeRequested) {
    debugln("in setOpmodeButtonColors, second pass");
    switch(requestedOpMode) {
      //**********************************************
      //***************** STANDBY *******************

      case STANDBY:
       bsby.Set_background_color_bco(YELLOW);
      break;

      case SENSORS_DEVELOPEMENT:
      break;

      case RUN_PRESET_COURSE:
      break;

      //**********************************************
      //***************** TELEOP *******************
      case TELEOP: // teleoperation by remote
       btele.Set_background_color_bco(YELLOW);
      break;

    //**********************************************
    //***************** FREE_RUN *******************
      case FREE_RUN:
       bfrun.Set_background_color_bco(YELLOW);
      break;

      //************************************************
      //******** MEASURE_AND_CALIBRATE_MOTORS **********
      case MEASURE_AND_CALIBRATE_MOTORS:
      break;
    
     default:
     ;
    }
  }
}

/*  Frome HMI page that sets the RTC
 * this function will read the combined states of
 * the following buttons and return a byte value
 * according to the following table
 * 
 * note: only one button in each row or collumn can be set
 * 
 *     9 year    10 month  12 day <   (btdate  8)
 *    17 hour    18 min    20 sec <   (bttime 16)
 *     ^          ^         ^        
 *   (btyh 1)  (btmm 2)  (btds 4)
 *   
 *
*/

byte getRTCbtnValue(void) {
  byte bv{0};
  uint32_t val{0};
  int del{25}; // delay to get good reading from HMI
  // check row (btyh 1)  (btmm 2)  (btds 4)
  // one only is set to 1 check row
  btyh.getValue(&val);
  if ( val == 1) {
    bv=1;
  }
  delay(del);
  btmm.getValue(&val);
  if ( val == 1) {
    bv=2;
  }
  delay(del);
  btds.getValue(&val);
  if ( val == 1) {
    bv=4;
  }

  // then check collumn
  // (btdate 8)
  // (bttime 16)
  // one only is set to 1
  delay(del);
  btdate.getValue(&val);
  if ( val == 1) {
    bv+=8;
  }
  delay(del);
  bttime.getValue(&val);
  if ( val == 1) {
    bv+=16;
  }
  return bv;
}

//get date and time from RTC input page and attempt to
//set RTC by it
void setRTCfromInput(void) {
  uint32_t nYear{0};
  uint32_t nMonth{0};
  uint32_t nDay{0};
  uint32_t nHour{0};
  uint32_t nMin{0};
  uint32_t nSec{0};
  int del{10}; // delay to get good reading from HMI

  nyear.getValue(&nYear);
  delay(del);
  nmonth.getValue(&nMonth);
  delay(del);
  nday.getValue(&nDay);
  delay(del);
  nhour.getValue(&nHour);
  delay(del);
  nmin.getValue(&nMin);
  delay(del);
  nsec.getValue(&nSec);
  // no check for valid date
  // input code does not permit overly wrong values for most dates except feb and user should
  // check on calendar for leap year

  if (rtcFound) {
   rtc.adjust(DateTime(nYear, nMonth, nDay, nHour, nMin, nSec));
   //debug rtc.adjust(DateTime(2017, 4, 21, 22, 52, 00));
  }
}

// gets date and time from RTC and set corresponding fields on page
void updateDTtoHMI(void) {
    now = rtc.now();
    dbSerialPrint("debug DT=");
    dbSerialPrintln(strDateTime(true));
    int del{10}; // delay to get good I/O fom HMI

    nyear.setValue(now.year());
    delay(del);
    nmonth.setValue(now.month());
    delay(del);
    nday.setValue(now.day());
    delay(del);

    nhour.setValue(now.hour());
    delay(del);
    nmin.setValue(now.minute());
    delay(del);
    nsec.setValue(now.second());
    delay(del);
}

// add +5|+1 to currently RTC selected field on page 2
void incrementField(int inc) {
  int del{10}; // delay to get good I/O fom HMI
  //  find currently selected field
  byte field = getRTCbtnValue();
  
  uint32_t val{0};
  uint32_t valm{0}; // needed to ckeck day

  //  ADD 5 to it, check and ajust to limits
  switch(field) {
    case 17: // hour
     // get current value
     nhour.getValue(&val);
     delay(10);
     nhour.getValue(&val);
     val+=inc;

     if (val >= 24) val = val % 24;
     delay(del);
     nhour.setValue(val);
    break;

    case 18: // minutes
     // get current value
     nmin.getValue(&val);
     delay(10);
     nmin.getValue(&val);
     val+=inc;

     if (val >= 60) val = val % 60;
     delay(del);
     nmin.setValue(val);
    break;

    case 20: // seconds
     // get current value
     nsec.getValue(&val);
     delay(10);
     nsec.getValue(&val);
     val+=inc;

     if (val >= 60) val = val % 60;
     delay(del);
     nsec.setValue(val);
    break;

    case 9: // year
     // get current value
     nyear.getValue(&val);
     delay(10);
     nyear.getValue(&val);
     val+=inc;

     if (val < 2020) val = 2020;
     delay(del);
     nyear.setValue(val);
    break;

    case 10: // month
     // get current value
     nmonth.getValue(&valm);
     delay(10);
     nmonth.getValue(&valm);
     valm+=inc;

     if (valm > 12) valm = valm % 12;
     delay(del);
     nmonth.setValue(valm);
    break;

    case 12: // day
     // get current day value
     nday.getValue(&val);
     delay(10);
     nday.getValue(&val);
     val+=inc;
     // get current month value
     nmonth.getValue(&valm);
     delay(10);
     nmonth.getValue(&valm);

     if (val > daysInMonth[valm-1]) val = 1;
     delay(del);
     nday.setValue(val);
    break;
 }
  
}

void bplus5PopCallback(void *ptr) {
  dbSerialPrintln("bplus5PopCallback");
  incrementField(5);
}

void bplus1PopCallback(void *ptr) {
  dbSerialPrintln("bplus1PopCallback");
  incrementField(1);
}

// subtract 1 from currently RTC selected field on page 2
void bminusPopCallback(void *ptr) {
  dbSerialPrintln("bminusPopCallback");
  // TODO:
  //  find currently selected field
  //  SUBSTRACT 1 from it, check and ajust to limits
  int del{10}; // delay to get good I/O fom HMI
  // TODO:
  // get current RTC and set fields to them or else set them to compile time
  
  //  find currently selected field
  byte field = getRTCbtnValue();
  uint32_t val{0};

  //  SUBSTRACT 1 from it, check and ajust to limits
  switch(field) {
    case 17: // hour
     // get current value
     nhour.getValue(&val);
     delay(10);
     nhour.getValue(&val);
     if (val !=0) --val;

     delay(del);
     nhour.setValue(val);
    break;

    case 18: // minutes
     // get current value
     nmin.getValue(&val);
     delay(10);
     nmin.getValue(&val);

     if (val !=0) --val;
     delay(del);
     nmin.setValue(val);
    break;

    case 20: // seconds
     // get current value
     nsec.getValue(&val);
     delay(10);
     nsec.getValue(&val);

     if (val !=0) --val;
     delay(del);
     nsec.setValue(val);
    break;

    case 9: // year
     // get current value
     nyear.getValue(&val);
     delay(10);
     nyear.getValue(&val);
     if (val !=0) --val;
     
     if (val < 2020) val = 2020;
     delay(del);
     nyear.setValue(val);
    break;
    
    case 10: // month
     // get current value
     nmonth.getValue(&val);
     delay(10);
     nmonth.getValue(&val);

     if (val == 0) val = 1;
     if (val > 1 ) --val;
     delay(del);
     nmonth.setValue(val);
    break;

    case 12: // day
     // get current value
     nday.getValue(&val);
     delay(10);
     nday.getValue(&val);

     if (val == 0) val = 1;
     if (val > 1) --val;
     delay(del);
     nday.setValue(val);
    break;

 }
}

void bsetRTCPopCallback(void *ptr) {
  dbSerialPrintln("bsetRTCPopCallback");
  setRTCfromInput();
  // validity of set date is somewhat garanteed
  // except for leap day
  // #TODO? feed back (highlight proprer fields) in case of incorrect value
}

// STANDBY opMode requested
void bsbyPopCallback(void *ptr)
{
  dbSerialPrintln("bsbyPopCallback");
  dbSerialPrintln("  ");
  if (currentOpMode != STANDBY) {
    if (!opModeChangeRequested) { //means first pression on button
     opModeChangeRequested = true;
     requestedOpMode = STANDBY; //set opMode desired
    }
      else if (requestedOpMode == STANDBY) {  //means second pression on button
        opModeChangeAutorized = true;
      }
    setOpmodeButtonColors(); // set opmode btn color according to new situation
  }
  else { // cancel any opmode change request
    opModeChangeRequested = false;
    page1.show(); // set opmode btn color according to new situation
    setOpmodeButtonColors();
  }
}

// TELEOP opMode requested
void btelePopCallback(void *ptr)
{
  dbSerialPrintln("btelePopCallback");
  dbSerialPrintln("  ");
  if (currentOpMode != TELEOP) { //means first pression on button
    if (!opModeChangeRequested) {
     opModeChangeRequested = true;
     requestedOpMode = TELEOP; //set opMode desired
    }
    else if (requestedOpMode == TELEOP) {  //means second pression on button
        opModeChangeAutorized = true;
      }
    setOpmodeButtonColors(); // set opmode btn color according to new situation
  }
  else { // cancel all opmode change variables
      opModeChangeRequested = false;
      page1.show(); // set opmode btn color according to new situation
      setOpmodeButtonColors();
  }
}

// FREE_RUN opMode requested
void bfrunPopCallback(void *ptr)
{
  dbSerialPrintln("btelePopCallback");
  dbSerialPrintln("  ");
  if (currentOpMode != FREE_RUN) { //means first pression on button
    if (!opModeChangeRequested) {
     opModeChangeRequested = true;
     requestedOpMode = FREE_RUN; //set opMode desired
    }
    else if (requestedOpMode == FREE_RUN) {  //means second pression on button
        opModeChangeAutorized = true;
      }
   setOpmodeButtonColors(); // set opmode btn color according to new situation
  }
  else { // cancel all opmode change variables
    opModeChangeRequested = false;
    page1.show(); // set opmode btn color according to new situation
    setOpmodeButtonColors();
  }
}
#endif
