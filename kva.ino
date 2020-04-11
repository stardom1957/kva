/*
 * 
 * KVA V1: first integration of Ps2 controler to replace joystick in TELEOP mode. In this version,
 *  - function motor_TELEOP_node_v1(): 
 *    - will handle an on the spot rotation by setting one motor to full speed and the opposite one to 0,
 *      wich meens that full X left or full X right stick value while Y stick is in at rest will produce no movement.
 *
 * KVA_V2: continuing integration. Changes for pin assignment for motorization.
 * 
 * 2019-06-23: projet created in my GitHub account. Pushed from KVA_V2 into ...Kookye_vehicule/kva.
 * History: 
 * ... will be maintained here and,
 * ... using issues, etc., on GitHub when mentionned
 * Issue 2.12 - Integrating and testing motor sensors through logic level shifter (8 ch):
 *  2.12.2 : 
 *   - DueTime encoderTimer is enabled during setup, but does not run if PWM signal is routed to pin 2 (ENB_R)
 *     - see new definition for ENB_R
 *   - all code for measuring motors is removed and modified to simply report motor encoder counts
 *     every time the program passes trough.
 * Issue 2.10.6: teleoperating -  emergency maneuvers
 *  - slow in place rotation using PS2 controler l/r buttons
 *  - slow forward/revers using PS2 controler up/down buttons
 */

// ************** Compile directives
#ifndef COMPILE_MAIN
#define COMPILE_MAIN
#endif

//#define DUE_TIMER_TEST0 // developpement test only, see DueTimer0 tab
//#define DUE_TIMER_TEST1 // developpement test only, see DueTimer1 tab
//#define COMPILE_PS2EXAMPLE // developpement test only
//#define JUMPERS_AS_INPUT // developpement only, setting opMode uing jumpers

//************************************************

#ifdef COMPILE_MAIN
#include <DueTimer.h>
#include "kva.h"

// robot vehicule modes of operation see vehiculeModes.txt
byte opMode;                             //operation mode
#define STANDBY 0                        //at rest but diagnostic and communication running
#define SENSORS_DEVELOPEMENT 15          // just run
#define JUST_RUN 30                      // just run
#define MEASURE_AND_CALIBRATE_MOTORS 40  //used to test what ever needs testing
#define TELEOP 10                        //Teleoperation with a joystick // TELEOP: Joystick operation
#define NADDOCAM 20                      //NADDOCAM: Non Directed Autonomous Driving with Obstacle Collision Avoidance mode:
/* NADDOCAM: Non Directed Autonomous Driving with Obstacle Collision Avoidance mode: 
 *  the vehicule navigates freely, begining driving forward from a start point while avoiding collisions.
*/

//************************************************
// defines and definitions for MEASURE_AND_CALIBRATE_MOTORS opMode only
//************************************************

#define ENCODER_MEASURE_INTERVAL 100000 // sampling interval = timer interval; set in microseconds (10e-6s)

//*******************************************
//************* motor control definitions
//*******************************************

// left motor on L298N channel A
#define ENA_L 4 // pwm pin
#define IN1   29
#define IN2   27

// right motor on L298N channel B
//#define ENB_R 2 // pwm pin
#define ENB_R 3 // pwm pin
#define IN3   25
#define IN4   23

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

const byte S1motorEncoder_L_PIN = 22;  // motor encoder S1 A pin
const byte S2motorEncoder_L_PIN = 24;  // motor encoder S2 B pin 
const byte S1motorEncoder_R_PIN = 26;  // motor encoder S1 pin
const byte S2motorEncoder_R_PIN = 28;  // motor encoder S2 pin

//*************************************************************
//definitions for Ps2 controler for teleoperation
//*************************************************************

#include <PS2X_lib.h>  //revised library from KurtE from Github
PS2X ps2x; // create PS2 Controller Class object

// SPI bus pins on Arduino Due ICSP connector near SAM3X8E chip
#define SPI_MISO 74
#define SPI_MOSI 75
#define SPI_CLK  76
#define PS2X_CS  49 // chip select for PS2X controler

int  PS2_config_result{254}; // controler never set = 254
byte PS2_type{0};
//byte PS2_vibrate_level{0}; // no vibration



//**************** FUNCTIONS DEFINITIONS
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

// ISR for the sensor counters
void ISR_S1_L(void) {
  ++S1_L_count;
}
void ISR_S1_R(void) {
  ++S1_R_count;
}

// Motor Speed Values - Start at zero

int motorSpeed_L = 0;
int motorSpeed_R = 0;
#define MOTOR_LOWER_PWM_LIMIT 25 // to avoid buzzing

// this set of motor functions are use for motor control in all
// the modes that require motor control by software.

// motor direction
const byte FORWARD {0};
const byte REVERSE {1};

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
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, HIGH);
     analogWrite(ENA_L, speed);
   }
   break;

  case REVERSE:
   // moteur droit recule
   if (speed >= MOTOR_LOWER_PWM_LIMIT && speed <= 255) {
     digitalWrite(IN1, HIGH);
     digitalWrite(IN2, LOW);
     analogWrite(ENA_L, speed);
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
    digitalWrite(ENA_L, LOW);
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
 
 PS2_config_result = ps2x.config_gamepad(SPI_CLK, SPI_MOSI, PS2X_CS, SPI_MISO, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for PS2_config_result
 
 if(PS2_config_result == 0){
   Serial.println("Found Controller, configured successful");
 }
   
  else if(PS2_config_result == 1)
   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(PS2_config_result == 2)
   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
   
  else if(PS2_config_result == 3)
   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
   
   //Serial.print(ps2x.Analog(1), HEX);
   
   PS2_type = ps2x.readType(); 
     switch(PS2_type) {
       case 0:
        Serial.println("Unknown Controller PS2_type");
       break;
       case 1:
        Serial.println("DualShock Controller Found");
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
  const int atRestZone {12};  // buffer zone to indicate stick is at rest in the middle
  
  if(PS2_config_result == 254) { //try to setup controler up to 10 times
   // debug Serial.println("Setting controler...");
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
 
  //ps2x.read_gamepad(false, PS2_vibrate_level);          //read controller and set large motor to spin at 'PS2_vibrate_level' speed
  ps2x.read_gamepad();

  //*************EMERGENCY ROTATION AND FORWARD/REVERSE ************
    
  /*
   PSB_PAD_UP      0x0010
   PSB_PAD_RIGHT   0x0020
   PSB_PAD_DOWN    0x0040
   PSB_PAD_LEFT    0x0080
   ps2x.ButtonPressed(PSB_RED)
   ps2x.ButtonReleased(PSB_PINK)
  */    
    int slow_motors {200};
    //slow forward
    if(ps2x.ButtonPressed(PSB_PAD_UP)){
      motorLeftSet(slow_motors, FORWARD);
      motorRightSet(slow_motors, FORWARD);
    }
    if(ps2x.ButtonReleased(PSB_PAD_UP)){
      motorAllStop();
    }

    //slow reverse
    if(ps2x.ButtonPressed(PSB_PAD_DOWN)){
      motorLeftSet(slow_motors, REVERSE);
      motorRightSet(slow_motors, REVERSE);
    }   
    if(ps2x.ButtonReleased(PSB_PAD_DOWN)){
      motorAllStop();
    }

    //slow left in place retation    
    if(ps2x.ButtonPressed(PSB_PAD_LEFT)){
      vehiculeRotateLeft(slow_motors);
    }
    if(ps2x.ButtonReleased(PSB_PAD_LEFT)) {
      motorAllStop();
    }

    //slow in place right rotation
    if(ps2x.ButtonPressed(PSB_PAD_RIGHT)){
       vehiculeRotateRight(slow_motors);
    }
    if(ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
      motorAllStop();
    }

  
  //**************** JOYSTICK OPERATION **********************
  //****************** left stick will be read only if button PSB_L1 is held pressed
  //****************** this is the dead man's grip
  if(ps2x.Button(PSB_L1)) {

    // reading the right stick values
    ps2RX = ps2x.Analog(PSS_RX); //Raw right stick X axis values are from 0 (full left) to 255 (full right), 128 is at rest in middle
    ps2RY = ps2x.Analog(PSS_RY); //Raw right stick Y axis values are from 0 (full up) to 255 (full down), 127 is at rest in middle

/* debug
    Serial.print(ps2RX, DEC); //Left stick, Y axis. Other options: LX, RY, RX  
    Serial.print(",");
    Serial.println(ps2RY, DEC); 
*/
/* PSS_RY determines if this is a forward or reverse motion
   FORWARD is PSS_RY <= (127 - atRestZone)
   REVERSE is PSS_RY >= (127 + atRestZone)
   
   Do this by reading the Verticle Value Y
   Apply results to MotorSpeed and to Direction
*/
   if (ps2RY >= (127 + atRestZone))
   {
     // This is reverse
     leftMotorDirection = REVERSE;
     rightMotorDirection = REVERSE;

     //motor speeds is determined from stick values ps2RY
     // we need to map the reading from 0 to 255

     motorSpeed_L = map(ps2RY, (127 + atRestZone), 255, 0, 255);
     motorSpeed_R = map(ps2RY, (127 + atRestZone), 255, 0, 255);
  }
  
  else if (ps2RY <= (127 - atRestZone))
  {
    // This is Forward
     leftMotorDirection = FORWARD;
     rightMotorDirection = FORWARD;

    //motor speeds is determined from stick values ps2RY
    //we need to map reading from 0 to 255

    motorSpeed_L = map(ps2RY, (127 - atRestZone), 0, 0, 255);
    motorSpeed_R = map(ps2RY, (127 - atRestZone), 0, 0, 255); 

  }
  else
  {
    // the stick is in the middle rest zone, so this is Stopped

    motorSpeed_L = 0;
    motorSpeed_R = 0; 

  }
  
  // Now do the steering
  // The Horizontal position X will "weigh" the motor speed
  // Values for each motor

   // PSS_LX determines the steering direction
   // LEFT  is PSS_LX <= (128 - atRestZone)
   // RIGHT is PSS_LX >= (128 + atRestZone)

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
    else { // if PSB_L1 not pressed, then we make sure motors are stopped
      motorAllStop();
    }
  delay(50);
} // fin motor_TELEOP_node_v1

void just_run() {
  Serial.println("Both motors FORWARD for 4s");
  motorLeftSet(200, FORWARD);
  motorRightSet(200, FORWARD);
  delay(4000);
  motorAllStop();
  Serial.println("Both motors STOPTED for 3s");
  delay(3000);
  Serial.println("Both motors REVERSE for 4s");
  motorLeftSet(200, REVERSE);
  motorRightSet(200, REVERSE);
  delay(4000);

  Serial.println("Left motor FORWARD and Right motor REVERSE for 4s");
  motorAllStop();
  motorLeftSet(200, FORWARD);
  motorRightSet(200, REVERSE);
  delay(4000);

  Serial.println("Left motor REVERSE and Right motor FORWARD for 4s");
  motorAllStop();
  motorLeftSet(200, REVERSE);
  motorRightSet(200, FORWARD);
  delay(4000);
     
  Serial.println("All stop for 2s");
  motorAllStop();
  delay(2000);
  Serial.println("_________");

  //while(true); //we're only doing this once
}


void measureAndCalibrateMotors(void) {
  Serial.println("Timer set for 10 readings per second. Delay is 6 sec.");
  motorLeftSet(128, FORWARD);
  motorRightSet(128, FORWARD);
  delay(6000); // run for n sec
  motorAllStop();
     
  // display results on Serial Monitor
   Serial.print("encoderTimerLoopCount= ");
  Serial.println(encoderTimerLoopCount, DEC);
  Serial.println("---------------------------");
  Serial.print("deltaCount_L= ");
  Serial.println(deltaCount_L, DEC);
  Serial.print("deltaCount_R= ");
  Serial.println(deltaCount_R, DEC);

  Serial.println("--------TOTALS------------\n");
  Serial.print("S1_L_count= ");
  Serial.println(S1_L_count, DEC);
  Serial.print("S1_R_count= ");
  Serial.println(S1_R_count, DEC);

  Serial.println("End of motor measure program.");

  while(true){
   ; // we do this only once
  }
}

// runs the selected opMode
void runOpMode(byte om) {
  switch (opMode) {
    case SENSORS_DEVELOPEMENT:
     sensorDeveloppement();
     break;

    case JUST_RUN:
     just_run();
     break;

    case TELEOP: // teleoperation by remote
     motor_TELEOP_node_v1();
     break;

    //**********************************************
    //***************** NADDOCAM *******************
    case NADDOCAM:
     break;

    //************************************************
    //***************** MEASURE_AND_CALIBRATE_MOTORS *
    case MEASURE_AND_CALIBRATE_MOTORS:
     measureAndCalibrateMotors();
     break;
    
    default:
     ; //do nothing for now
  }  
}


void setup()
{
  //debug noInterrupts(); //no interrupts at this point

  //debug we might use this later
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.begin(115200);

  // set all the motor control pins to outputs
  pinMode(ENA_L, OUTPUT);
  pinMode(ENB_R, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // set all encoder sensors to INPUT
  pinMode(S1motorEncoder_L_PIN,INPUT);
  pinMode(S2motorEncoder_L_PIN,INPUT);
  pinMode(S1motorEncoder_R_PIN,INPUT);
  pinMode(S2motorEncoder_R_PIN,INPUT);

  motorAllStop();

  pinMode(PS2X_CS, OUTPUT); //PS2 controler chip select pin

  attachInterrupt(S1motorEncoder_L_PIN, ISR_S1_L, CHANGE);
  attachInterrupt(S1motorEncoder_R_PIN, ISR_S1_R, CHANGE);

  //setup timer interrupt for motor encoders
  encoderTimer.setPeriod(ENCODER_MEASURE_INTERVAL); // in microseconds
  encoderTimer.attachInterrupt(ISR_timerEncoder);
  encoderTimer.start();
  //debug interrupts(); // allow interrupt starting here

 // init of message structures
 msgRingBuffer[0].createMsg(COLLISION_SENSOR_STATUS, "5", 1);
 msgRingBuffer[1].createMsg(CURRENT_OP_MODE, String(SENSORS_DEVELOPEMENT), 1);

 currentMsg=0;
 Serial.println(msgRingBuffer[0].msg);
 Serial.println(msgRingBuffer[0].topic);
 Serial.println(msgRingBuffer[0].priority);
  
 Serial.println(msgRingBuffer[1].msg);
 Serial.println(msgRingBuffer[1].topic);
 Serial.println(msgRingBuffer[1].priority);

}

void loop()
{
 //****************************************************************
 //***************** get new opMode value from command channel ****
 //****************************************************************
 // for now hard code the opMode

 //opMode = TELEOP;
 //opMode = MEASURE_AND_CALIBRATE_MOTORS;
 //opMode = JUST_RUN;
 //opMode = STANDBY
 opMode = SENSORS_DEVELOPEMENT;
 //opMode = NADDOCAM;
 
 runOpMode(opMode);

/*

*/
}
//endif for COMPILE
#endif

