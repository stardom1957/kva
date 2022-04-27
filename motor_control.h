#ifndef _motor_control_h
#define _motor_control_h
#ifdef MOTOR_CONTROL_COMPILE

void ISR_timerEncoder(void);
void ISR_S1_L(void);
void ISR_S1_R(void);
void motorRightSet(int, byte);
void vehiculeRotateRight(int);
void vehiculeRotateLeft(int);
void motorLeftSet(int, byte);
void motorRightStop(void);
void motorLeftStop(void);
void motorAllStop(void);

//ISR timer for motor sensor readings
void ISR_timerEncoder(void) {
     //debug noInterrupts(); //stop all interrupts
     //debug possibly just detach this ISR here and reattach at the end

     //S1_L_count for timer period deltaCount_L = S1_L_count - S1_L_count_previous;
     deltaCount_L = S1_L_count - S1_L_count_previous;
     S1_L_count_previous = S1_L_count;

     //S1_R_count for timer period deltaCount_R = S1_R_count - S1_R_count_previous;
     deltaCount_R = S1_R_count - S1_R_count_previous;
     S1_R_count_previous = S1_R_count;
     ++encoderTimerLoopCount;
     // debug interrupts(); //restart all interupts
}

// ISR for the motor sensor counters
void ISR_S1_L(void) {
  ++S1_L_count;
}

void ISR_S1_R(void) {
  ++S1_R_count;
}

// this set of motor functions are use for motor control in all
// the modes that require motor control by software.

void motorRightSet(int speed, byte direction) {
 switch (direction) {
  case FORWARD:
   // moteur droit avance
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
     analogWrite(ENB_R, speed);
   }
   break;

  case REVERSE:
   // moteur droit recule
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN3, HIGH);
     digitalWrite(IN4, LOW);
     analogWrite(ENB_R, speed);
   }
   break;
   
   default:
     ; //nothing
 }
}

// rotate the vehicule in place l and r
void vehiculeRotateRight(int speed) {
 motorRightSet(speed, REVERSE);
 motorLeftSet(speed, FORWARD);
}

void vehiculeRotateLeft(int speed) {
 motorRightSet(speed, FORWARD);
 motorLeftSet(speed, REVERSE);
}

void motorLeftSet(int speed, byte direction) {
 switch (direction) {
  case FORWARD:
   // moteur droit avance
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN1_PIN, LOW);
     digitalWrite(IN2_PIN, HIGH);
     analogWrite(ENA_L_PIN, speed);
   }
   break;

  case REVERSE:
   // moteur droit recule
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN1_PIN, HIGH);
     digitalWrite(IN2_PIN, LOW);
     analogWrite(ENA_L_PIN, speed);
   }
   break;

   default:
     ; //nothing
 }
}

void motorRightStop(void) {
    digitalWrite(ENB_R, LOW);
}

void motorLeftStop(void) {
    digitalWrite(ENA_L_PIN, LOW);
}

void motorAllStop(void)
{
  motorLeftStop();
  motorRightStop();
}


#endif //MOTOR_CONTROL_COMPILE
#endif
