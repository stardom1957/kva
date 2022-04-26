#ifdef DUE_TIMER_TEST1
#include <DueTimer.h>
#define ENCODER_MEASURE_INTERVAL 100000 // sampling interval = timer interval; set in microseconds (10e-6s)

//*************** motors encoders (Hall) sensors definitions
//setup timer interrupt for motor encoders
DueTimer encoderTimer = Timer.getAvailable();

volatile unsigned long S1_L_count {0};          // running count for Hall sensor S1, left motor
volatile unsigned long S1_L_count_previous {0}; // previous count for Hall sensor S1, left motor
volatile unsigned long deltaCount_L {0};        // number of counts for Hall sensor S1, left motor for measuring period

volatile unsigned long S1_R_count {0};          // running count for Hall sensor S1, right motor
volatile unsigned long S1_R_count_previous {0}; // previous count for Hall sensor S1, right motor
volatile unsigned long deltaCount_R {0};        // number of counts for Hall sensor S1, right motor for measuring period

volatile unsigned long encoderTimerLoopCount {0};     // number of passes trough timer

void ISR_timerEncoder(void) {
     //debug noInterrupts(); //stop all interrupts

     //S1_L_count for timer period deltaCount_L = S1_L_count - S1_L_count_previous;
     deltaCount_L = S1_L_count - S1_L_count_previous;
     S1_L_count_previous = S1_L_count;

     //S1_R_count for timer period deltaCount_R = S1_R_count - S1_R_count_previous;
     deltaCount_R = S1_R_count - S1_R_count_previous;
     S1_R_count_previous = S1_R_count;
     ++encoderTimerLoopCount;
     //debug interrupts(); //restart all interupts
}


void setup(){
  Serial.begin(115200);
  //setup timer interrupt for motor encoders
  encoderTimer.setPeriod(ENCODER_MEASURE_INTERVAL); // in microseconds
  encoderTimer.attachInterrupt(ISR_timerEncoder);
  encoderTimer.start();
}

void loop(){
     Serial.println("Timer set for 10 readings per second. Start of measure program for 2 sec.");
     //motorRightForward(255);
     //motorLeftForward(255);
     delay(2000); // simulate run for 2 sec
     
     // display results on Serial Monitor
/*
volatile unsigned long S1_L_count {0};          // running count for Hall sensor S1, left motor
volatile unsigned long S1_L_count_previous {0}; // previous count for Hall sensor S1, left motor
volatile unsigned long deltaCount_L             // number of counts for Hall sensor S1, left motor for measuring period
*/    
     debug("encoderTimerLoopCount= ");
     debugln__2arg(encoderTimerLoopCount, DEC);
     Serial.println("---------------------------");
     debug("deltaCount_L= ");
     debugln__2arg(deltaCount_L, DEC);
     debug("deltaCount_R= ");
     debugln__2arg(deltaCount_R, DEC);

//     encoderTimer.stop();
//     motorAllStop();
     Serial.println("End of measure program.");

     while(true){
      ; // we do this only once
     }
}
#endif
