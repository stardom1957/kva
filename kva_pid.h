#ifndef _kva_pid_h
#define _kva_pid_h 
/*
from ArduinoTutorials (github). At curiores@gmail.com
adapted from SimplePositionPID_NoAtomic.ino
*/
#include <Wire.h>

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
    dir = 1;
    if (u < 0) {
      dir = -1;
    }

    // store previous error
    eprev = e;
   }
};


// Globals
long prevTime{0};
volatile int posi[] = {0,0};

// PID class instances
SimplePID pid[2];

#endif
