#ifndef _hmi_h
#define _hmi_h

/**
 Notes:
 1. No actual modificationto nexConfig.h
 
 */

#include "Nextion.h"
// pages
NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");

// pgStatus components
NexText topmode          = NexText(0, 5, "topmode");          // displays current opMode
NexProgressBar jbuffstat = NexProgressBar(0, 6, "jbuffstat");  // displays progress bar for message buffer status
NexButton bopmode        = NexButton(0, 3, "bopmode");    // button to opMode selection page

// pgOpMode components
NexButton bsby      = NexButton(1, 3, "bsby");             // STANDBY mode select button
NexButton btele     = NexButton(1, 5, "btele");           // TELEOPY mode select button
NexButton bfrun     = NexButton(1, 4, "bfrun");          // FREE_RUN mode select button
NexNumber ngotimer  = NexNumber(1, 7, "ngotimer");            // time count for timer to start selected opMode
NexButton bstat     = NexButton(1, 2, "bstat");       // button to status page


char char_buffer[25] = {0};              // buffer to hold data from and to HMI
unsigned long timerOm{0};                // timer count to control delay to start selected opMode via HMI
unsigned long displayTimer{0};           // timer count to control dislay of status to HMI
unsigned long displayInterval{1000};     // display status interval in ms
uint32_t delayStatOpMode = (uint32_t)10; // delay in s
uint32_t ngotimer_val = delayStatOpMode; // delay to start opMode

//***************************************
//***************************************
// pgStatus components call back functions

// page change ****************
// call back of bopmode button
void bopmodePopCallback(void *ptr)
{
    page1.show();
    //debug info to serial
    //NexButton *btn = (NexButton *)ptr;
    dbSerialPrintln("bopmodePopCallback");
    //dbSerialPrint("ptr=");
    //dbSerialPrintln((uint32_t)ptr);

    // get the page name the HMI is on
    //for the moment this is a place holder
    dbSerialPrintln("get current HMI page = later");
    dbSerialPrintln("  ");
}

// page change ****************
// call back of bstat
void bstatPopCallback(void *ptr)
{
  page0.show();
  dbSerialPrintln("bstatPopCallback");
  dbSerialPrintln("get current HMI page = later");
  dbSerialPrintln("  ");

}

//***************************************
//***************************************
// pgOpMode components call back functions


// call back of bsby select STANDBY opMode
void bsbyPopCallback(void *ptr)
{
  // set attribute of btele button to show not selected
  // set attribute of bfrun button to show not selected
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
  // set attribute of bsby button to show not selected
  // set attribute of bfrun button to show not selected
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
  // set attribute of bsby button to show not selected
  // set attribute of btele button to show not selected
  // reset delay timer
  dbSerialPrintln("bfrunPopCallback");
  dbSerialPrintln("  ");
  //ngotimer.setValue(delayStatOpMode);
  //timerOm = millis(); // start timer
  selectedOpMode = FREE_RUN; //set opMode desired
  opModeChangeRequested = true;
}

// register components
NexTouch *nex_listen_list[] = 
{
    &bopmode,
    &bsby,
    &btele,
    &bfrun,
    &bstat,
    &page0,
    &page1,
    NULL
};


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
