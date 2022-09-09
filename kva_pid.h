#ifndef _kva_pid_h
#define _kva_pid_h 
/*
from ArduinoTutorials (github). At curiores@gmail.com
adapted from SimplePositionPID_NoAtomic.ino
*/
#include <DATASCOPE.h>
#include <DueTimer.h>

//******************************************************
// definitions for measure interval of timer
//******************************************************
#define ENCODER_MEASURE_INTERVAL 10000 // sampling interval = timer interval; set in microseconds (10e-6s)

//setup timer interrupt for motor encoders
DueTimer encoderTimer = Timer.getAvailable();

volatile unsigned long S1_L_count {0};          // running count for Hall sensor S1, left motor
volatile unsigned long S1_L_count_previous {0}; // previous count for Hall sensor S1, left motor
volatile unsigned long deltaCount_L {0};        // number of counts for Hall sensor S1, left motor for measuring period

volatile unsigned long S1_R_count {0};          // running count for Hall sensor S1, right motor
volatile unsigned long S1_R_count_previous {0}; // previous count for Hall sensor S1, right motor
volatile unsigned long deltaCount_R {0};        // number of counts for Hall sensor S1, right motor for measuring period

volatile unsigned long encoderTimerLoopCount {0};     // number of passes trough timer

//ISR timer for motor sensor readings
void ISR_timerEncoder(void) {
     //debug noInterrupts(); //stop all interrupts
     //debug possibly just detach this ISR here and reattach at the end

     //S1_L_count for timer period
     deltaCount_L = S1_L_count - S1_L_count_previous;
     S1_L_count_previous = S1_L_count;

     //S1_R_count for timer period
     deltaCount_R = S1_R_count - S1_R_count_previous;
     S1_R_count_previous = S1_R_count;
     ++encoderTimerLoopCount;
     // debug interrupts(); //restart all interupts
}

// class definition
class SimplePID {
  private:
   // PID constant for propotionnal, devivative and integral
   float kp, kd, ki, umax;
   float eprev, eintegral; // previous error, integral error

  public:
   // constructor
   SimplePID(): kp(1.0), kd(0.0), ki(0.0), umax(255), eprev(0.0), eintegral(0.0){}

   // set the PID parameters
   void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
   }

   // Evaluate the signal
   void evalu(int valueRead, int target, float deltaTime, int &pwr, int &dir) {
    // calculate error between valueRead and target (set point)
    int e = target - valueRead;

    // derivative component
    float dedt = (e - eprev)/deltaTime;

    // integral component
    eintegral = eintegral + e * deltaTime;
    
    // calculate PID
    float u = kp*e + kd*dedt + ki*eintegral;

    // calculate new motor power
    pwr = (int)fabs(u);
    if (pwr > umax) {
      pwr = umax;
    }

    // determine direction of correction
    //dir = 1;
    dir = FORWARD;
    if (u < 0) {
      //dir = -1;
      dir = BACKWARD;
    }

    // store previous error
    eprev = e;
   }
};

/*
const byte S1motorEncoder_L_PIN = 22;  // motor encoder S1 A pin
const byte S2motorEncoder_L_PIN = 24;  // motor encoder S2 B pin 
const byte S1motorEncoder_R_PIN = 26;  // motor encoder S1 pin
const byte S2motorEncoder_R_PIN = 28;  // motor encoder S2 pin

*/

// globals
long prevTime{0}; // prevousTime
volatile int posi_L{0}; //  left encoder position (count)
volatile int posi_R{0}; // right encoder position (count)

// interrupt service routines
// read left motor S2 encoder
void ISR_readEncoder_L(){
  int b = digitalRead(S2motorEncoder_L_PIN);
  if(b == HIGH){
    posi_L++; // FORWARD
  }
  else{
    posi_L--; // BACKWARD
  }
}

// read right motor S2 encoder
void ISR_readEncoder_R(){
  int b = digitalRead(S2motorEncoder_R_PIN);
  if(b == LOW){
    posi_R++; // FORWARD
  }
  else{
    posi_R--; // BACKWARD
  }
}

// PID class instances
 SimplePID pid_L;
 SimplePID pid_R;


#endif
