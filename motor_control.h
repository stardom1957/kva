#ifndef _motor_control_h
#define _motor_control_h
#ifdef MOTOR_CONTROL_COMPILE

// status for motor control
int motorSpeed_L = 0; // Motor Speed Values - Start at zero
int motorSpeed_R = 0;
#define MOTOR_LOWER_PWM_LIMIT 25 // to avoid buzzing
#define EMERGENCY_SLOW 150 //emergency speed to get out of trouble


//*******************************************
//************* L298N ***********************
//************* motor control definitions ***
//*******************************************

// left motor on L298N channel A
#define ENA_L_PIN 4 // enable PWM speed control
#define IN1_L_PIN 29 // direction control
#define IN2_L_PIN 27 // direction control

// right motor on L298N channel B
#define ENB_R     3 // enable PWM speed control
#define IN3_R_PIN 25 // direction control
#define IN4_R_PIN 23 // direction control

//*************** motors encoders (Hall) sensors definitions
#define S1motorEncoder_L_PIN 22  // motor encoder S1 A pin
#define S2motorEncoder_L_PIN 24  // motor encoder S2 B pin 
#define S1motorEncoder_R_PIN 26  // motor encoder S1 A pin
#define S2motorEncoder_R_PIN 28  // motor encoder S2 B pin

//setup timer interrupt for motor encoders
DueTimer encoderTimer = Timer.getAvailable();

volatile unsigned long S1_L_count {0};          // running count for Hall sensor S1, left motor
volatile unsigned long S1_L_count_previous {0}; // previous count for Hall sensor S1, left motor
volatile unsigned long deltaCount_L {0};        // number of counts for Hall sensor S1, left motor for measuring period
volatile unsigned long S1_R_count {0};          // running count for Hall sensor S1, right motor
volatile unsigned long S1_R_count_previous {0}; // previous count for Hall sensor S1, right motor
volatile unsigned long deltaCount_R {0};        // number of counts for Hall sensor S1, right motor for measuring period
volatile unsigned long encoderTimerLoopCount {0};     // number of passes trough timer

// motor direction
const byte FORWARD {0};
const byte BACKWARD {1};

// ISRs function prototypes
void ISR_timerEncoder(void); // reads motors sensors
void ISR_S1_L(void); // maintains left motor sensor count
void ISR_S1_R(void); // maintains right motor sensor count

// motor control functions prototypes
void motorRightSet(int, byte);
void vehiculeRotateRight(int);
void vehiculeRotateLeft(int);
void motorLeftSet(int, byte);
void motorRightStop(void);
void motorLeftStop(void);
void motorAllStop(void);

// ISR for motor sensor readings
// this ISR is attached to encoderTimer
// reads and updates count values
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
     digitalWrite(IN3_R_PIN, LOW);
     digitalWrite(IN4_R_PIN, HIGH);
     analogWrite(ENB_R, speed);
   }
   break;

  case BACKWARD:
   // moteur droit recule
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN3_R_PIN, HIGH);
     digitalWrite(IN4_R_PIN, LOW);
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

void motorLeftSet(int speed, byte direction) {
 switch (direction) {
  case FORWARD:
   // moteur droit avance
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN1_L_PIN, LOW);
     digitalWrite(IN2_L_PIN, HIGH);
     analogWrite(ENA_L_PIN, speed);
   }
   break;

  case BACKWARD:
   // moteur droit recule
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN1_L_PIN, HIGH);
     digitalWrite(IN2_L_PIN, LOW);
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

/* this function implement the TELEOP mode: the vehicule movement is fully controled by a remote operator using a Ps2 type controler. 
 *  This mode doesn't provide for any type of collision avoidance.

*/

//***************** PS2 controler setting function
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

void motor_TELEOP_node_v1(void) {
  byte leftMotorDirection;
  byte rightMotorDirection;
  int ps2RY; //we are using the right hand joystick
  int ps2RX;  
  //const int atRestZone {12};  // buffer zone to indicate stick is at rest in the middle
  const int atRestZone {3};  // buffer zone to indicate stick is at rest in the middle
  boolean emergency_run{false}; // indicates we are moving using emergency buttons
  
  if(PS2_config_result == 254) { //try to setup controler up to 10 times
   // debug debugln("Setting controler...");
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

   if (ps2RY >= (127 + atRestZone)) {
     // This is reverse
     leftMotorDirection = BACKWARD;
     rightMotorDirection = BACKWARD;

     //motor speeds is determined from stick values ps2RY
     // we need to map the reading from 0 to 255

     motorSpeed_L = map(ps2RY, (127 + atRestZone), 255, 0, 255);
     motorSpeed_R = map(ps2RY, (127 + atRestZone), 255, 0, 255);
   }
  
   else if (ps2RY <= (127 - atRestZone)) {
    // This is Forward
     leftMotorDirection = FORWARD;
     rightMotorDirection = FORWARD;

    //motor speeds is determined from stick values ps2RY
    //we need to map reading from 0 to 255

    motorSpeed_L = map(ps2RY, (127 - atRestZone), 0, 0, 255);
    motorSpeed_R = map(ps2RY, (127 - atRestZone), 0, 0, 255); 

   }
   else {
    // the stick is in the middle rest zone, so this is Stopped
    motorSpeed_L = 0;
    motorSpeed_R = 0; 
   }

  /*
   Now do the steering
   The Horizontal position X will "weigh" the motor speed
   Values for each motor

   ps2RX determines the steering direction
   LEFT  is ps2RX <= (128 - atRestZone)
   RIGHT is ps2RX >= (128 + atRestZone)
   */

  if (ps2RX <= (128 - atRestZone)) {
    // We move to the left
    // Map the number to a value of 255 maximum

    ps2RX = map(ps2RX, 0, (128 - atRestZone), 255, 0);
    motorSpeed_L = motorSpeed_L - ps2RX;
    motorSpeed_R = motorSpeed_R + ps2RX;

    // Don't exceed range of 0-255 for motor speeds
    if (motorSpeed_L < 0) motorSpeed_L = 0;
    if (motorSpeed_R > 255) motorSpeed_R = 255;
  }
    else if (ps2RX >= (128 + atRestZone)) {
    // we move to the right
    // Map the number to a value of 255 maximum

      ps2RX = map(ps2RX, (128 + atRestZone), 255, 0, 255);
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
} // fin motor_TELEOP_node_v1


#endif //MOTOR_CONTROL_COMPILE
#endif
