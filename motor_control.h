#ifndef _motor_control_h
#define _motor_control_h
#ifdef MOTOR_CONTROL_COMPILE

// function prototypes
void motorRightSet(int, byte);
void vehiculeRotateRight(int);
void vehiculeRotateLeft(int);
void motorLeftSet(int, byte);
void motorRightStop(void);
void motorLeftStop(void);
void motorAllStop(void);

void motorLeftSet(int speed, byte direction) {
 switch (direction) {
  case FORWARD:
   // moteur avance
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN1_PIN, LOW);
     digitalWrite(IN2_PIN, HIGH);
     analogWrite(ENA_L_PIN, speed);
   }
   break;

  case BACKWARD:
   // moteur recule
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

  case BACKWARD:
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
 motorRightSet(speed, BACKWARD);
 motorLeftSet(speed, FORWARD);
}

void vehiculeRotateLeft(int speed) {
 motorRightSet(speed, FORWARD);
 motorLeftSet(speed, BACKWARD);
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

/* this function implement the TELEOP mode: the vehicule movement is fully controled by a remote operator using a Ps2 type controler. 
 *  This mode doesn't provide for any type of collision avoidance.

*/

//***************** PS2 controler setting function
// status for TELEOP mode
int  PS2_config_result{254}; // controler never set = 254
byte PS2_type{0};

void set_ps2x(void) {
 digitalWrite(PS2X_CS, LOW); // select controler for action
 //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for PS2_config_result
 PS2_config_result = ps2x.config_gamepad(SPI_CLK, SPI_MOSI, PS2X_CS, SPI_MISO, true, true);
 
 if(PS2_config_result == 0){
   debugln("Found Controller, configured successful");
 }
   
  else if(PS2_config_result == 1)
   debugln("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(PS2_config_result == 2)
   debugln("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
   
  else if(PS2_config_result == 3)
   debugln("Controller refusing to enter Pressures mode, may not support it. ");
   
   //debugln__2arg(ps2x.Analog(1), HEX);
   
   PS2_type = ps2x.readType(); 
     switch(PS2_type) {
       case 0:
        debugln("Unknown Controller PS2_type");
       break;
       case 1:
        debugln("DualShock Controller Found");
       break;
     }
}


//********************************************
//***************** TELEOP *******************

void motor_TELEOP(void) {
  byte leftMotorDirection;
  byte rightMotorDirection;
  int ps2RY; //we are using the right hand joystick
  int ps2RX;  
  //const int atRestZone {12};  // buffer zone to indicate stick is at rest in the middle
  const int atRestZone {3};     // buffer zone to indicate stick is at rest in the middle
  
  /*
    emergency_run indicate we are moving using emergency buttons: triangle, square, circle and cross.
  */
  
  boolean emergency_run{false};

  if (run_setup) {
    currentOpModeName = "TELEOP"; // label of this mode in page 0 of HMI
    run_setup = false;

  }

  if(PS2_config_result == 254) { //try to setup controler up to 10 times
    debugln("Setting controler...");
    byte sc;
    for (sc=0; sc<10; sc++) {
      set_ps2x();
      if (PS2_config_result == 0) break;
      delay(50);
    }
  }

  // if no controler was found on last run, then ensure we will attempt to setup controler in next run
  if (PS2_config_result != 0 || PS2_type != 1) {
    PS2_config_result == 254;
    return;
  }
 
  ps2x.read_gamepad();
  //****************** all buttons are read only if button PSB_L1 is held pressed
  //****************** this is the dead man's grip

  if(ps2x.Button(PSB_L1)) {
    //*************EMERGENCY ROTATION AND FORWARD/BACKWARD ************
    // using controller triangle, cross, square and circle
    //slow forward
    if(ps2x.ButtonPressed(PSB_TRIANGLE)){
      debugln("emergency forward");
      emergency_run = true;
      motorLeftSet(EMERGENCY_SLOW, FORWARD);
      motorRightSet(EMERGENCY_SLOW, FORWARD);
    }
    if(ps2x.ButtonReleased(PSB_TRIANGLE)){
      motorAllStop();
      emergency_run = false;
    }

    //slow reverse
    if(ps2x.ButtonPressed(PSB_CROSS)){
      emergency_run = true;
      motorLeftSet(EMERGENCY_SLOW, BACKWARD);
      motorRightSet(EMERGENCY_SLOW, BACKWARD);
    }   
    if(ps2x.ButtonReleased(PSB_CROSS)){
      emergency_run = false;
      motorAllStop();
    }

    //slow left in place rotation    
    if(ps2x.ButtonPressed(PSB_SQUARE)){
      emergency_run = true;
      vehiculeRotateLeft(EMERGENCY_SLOW);
    }
    if(ps2x.ButtonReleased(PSB_SQUARE)) {
      emergency_run = false;
      motorAllStop();
    }

    //slow in place right rotation
    if(ps2x.ButtonPressed(PSB_CIRCLE)){
       emergency_run = true;
       vehiculeRotateRight(EMERGENCY_SLOW);
    }
    if(ps2x.ButtonReleased(PSB_CIRCLE)) {
      emergency_run = false;
      motorAllStop();
    }

  
  //**************** JOYSTICK OPERATION ********************************************

   // reading the right stick values
   ps2RX = ps2x.Analog(PSS_RX); //Raw right stick X axis values are from 0 (full left) to 255 (full right), 128 is at rest in middle
   ps2RY = ps2x.Analog(PSS_RY); //Raw right stick Y axis values are from 0 (full up) to 255 (full down), 127 is at rest in middle

    /*
      motor speeds is determined from stick values ps2RY, so
      we need to map the reading from 0 to 255 and set BOTH
      motors to that speed.

    */

   // This is reverse
   if (ps2RY >= (127 + atRestZone)) {
     leftMotorDirection = BACKWARD;
     rightMotorDirection = BACKWARD;

     //             map(value, fromLow, fromHigh, toLow, toHigh)
     motorSpeed_L = map(ps2RY, (127 + atRestZone), 255, 0, 255);
     motorSpeed_R = motorSpeed_L;
   }
  
   // this is forward
   else if (ps2RY <= (127 - atRestZone)) {
     leftMotorDirection = FORWARD;
     rightMotorDirection = FORWARD;

    //             map(value, fromLow, fromHigh, toLow, toHigh)
    motorSpeed_L = map(ps2RY, (127 - atRestZone), 0, 0, 255);
    motorSpeed_R = motorSpeed_L;
   }
   else {
    // the stick is in the middle rest zone, so this is Stopped
    motorSpeed_L = 0;
    motorSpeed_R = 0; 
   }

  /* Now do the steering
   
   The Horizontal position X will "weigh" the motor speed values for each motor.

   ps2RX determines the steering direction with:
     LEFT: ps2RX <= (128 - atRestZone)
    RIGHT: ps2RX >= (128 + atRestZone)

   Raw X axis values are from 0 (full left) to 255 (full right) and
   128 is at rest in middle.

   */

  if (ps2RX <= (128 - atRestZone)) {
    /*
     We move to the left
     Map the number to a value of 255 maximum
     
    */
    //debug ps2RX = map(ps2RX, 0, (128 - atRestZone), 255, 0);
    //      map(value, fromLow, fromHigh, toLow, toHigh)
    ps2RX = map(ps2RX, 0, 128, 128, 0);

    //ps2RX = map(ps2RX, 0, 128, 255, 0);
    motorSpeed_L = motorSpeed_L - ps2RX;
    motorSpeed_R = motorSpeed_R + ps2RX;

    // Don't exceed range of 0-255 for motor speeds
    if (motorSpeed_L < 0) motorSpeed_L = 0;
    if (motorSpeed_R > 255) motorSpeed_R = 255;
  }
    else if (ps2RX >= (128 + atRestZone)) {
    /*
     we move to the right
     Map the number to a value of 255 maximum
    
    */
      //ps2RX = map(ps2RX, (128 + atRestZone), 255, 0, 255);
      //      map(value, fromLow, fromHigh, toLow, toHigh)
      ps2RX = map(ps2RX, 128, 255, 0, 128);

      //ps2RX = map(ps2RX, 128, 255, 0, 255);
      motorSpeed_L = motorSpeed_L + ps2RX;
      motorSpeed_R = motorSpeed_R - ps2RX;

    // Don't exceed range of 0-255 for motor speeds
      if (motorSpeed_L > 255) motorSpeed_L = 255;
      if (motorSpeed_R < 0) motorSpeed_R = 0;      
    }

  // Adjust to prevent "buzzing" at very low speed

  if (motorSpeed_L < MOTOR_LOWER_PWM_LIMIT) motorSpeed_L = 0;
  if (motorSpeed_R < MOTOR_LOWER_PWM_LIMIT) motorSpeed_R = 0;

  // get the motors going
  motorLeftSet(motorSpeed_L, leftMotorDirection);
  motorRightSet(motorSpeed_R, rightMotorDirection);

  }
  else { // if PSB_L1 not pressed, then we stop the motors
        motorAllStop();
  }
  //debug delay(50);
  // debug run_setup = false;
} // fin motor_TELEOP


#endif //MOTOR_CONTROL_COMPILE
#endif
