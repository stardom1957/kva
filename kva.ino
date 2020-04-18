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
 *     - see new definition for ENB_R OK
 *   - all code for measuring motors is removed and modified to simply report motor encoder counts
 *     every time the program passes trough. DONE
 * Issue 2.10.6: teleoperating -  emergency maneuvers DONE
 *  - slow in place rotation using PS2 controler l/r buttons DONE
 *  - slow forward/revers using PS2 controler up/down buttons DONE
 * 
 * Issue 6.2.1 Software (class) developement
 *  - mesage passing routines see kva.h
 * 
 * Issue 6.3: On board displays and controls using Nextion HMI
 *  - Nextion library
 *  - Nextion HMI page menu for setting opMode
 *  - Nextion HMI page for STATUS
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
#define SERIAL_DEBUG_PORT     0 // serial port for debuging
#define SERIAL_TELEMETRY_PORT 1 // serial port to XBee RF module
#define SERIAL_HMI            2 // serial port to Nextion HMI

boolean opModeChangeRequested {false};
#include <DueTimer.h>
#include "kva.h"

// robot vehicule modes of operation see vehiculeModes.txt
byte currentOpMode; // operation mode
byte selectedOpMode;                     // user selected opmode
String currentOpModeName;                // text value of current opMode name 
#define STANDBY 0                        // at rest but diagnostic and communication running
#define SENSORS_DEVELOPEMENT 15          // as it says
#define JUST_DO_THIS 30                  // executes a series of preset commands
#define FREE_RUN  35                     // runs in obstacle collision avoidance on
#define MEASURE_AND_CALIBRATE_MOTORS 40  // used to test what ever needs testing
#define TELEOP 10                        // Teleoperation with a joystick // TELEOP: Joystick operation

#include "hmi.h" // for HMI display and control

/* FREE_RUN: Non Directed Autonomous Driving with Obstacle Collision Avoidance: 
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
#define ENA_L_PIN 4 // pwm pin
#define IN1_PIN   29
#define IN2_PIN   27

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

//*************************************************************
//definitions for displays, indicators, etc.
//*************************************************************
#define MESSAGE_BUFFER_OVERFLOW_LED_PIN 53

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
  updateDisplayAndIndicators();
  
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

void just_do_this() {
  updateDisplayAndIndicators();
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
  Serial.println("___ wait 5 s______");

  //while(true); //we're only doing this once
  delay(5000);
}

//debug note SHOULD BE REMOVED (BY COMMENTING) IN PRODUCTION
void measureAndCalibrateMotors(void) {
  updateDisplayAndIndicators();
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
  switch (om) {
    case STANDBY:
     currentOpModeName = "STANDBY";
     standby();
     break;

    case SENSORS_DEVELOPEMENT:
     currentOpModeName = "SENS. DEVEL.";
     sensorDeveloppement();
     break;

    case JUST_DO_THIS:
     currentOpModeName = "JUST_DO_THIS";
     just_do_this();
     break;

    case TELEOP: // teleoperation by remote
     currentOpModeName = "TELEOP";
     motor_TELEOP_node_v1();
     break;

    //**********************************************
    //***************** FREE_RUN *******************
    case FREE_RUN:
     currentOpModeName = "FREE RUN";
     free_run();
     break;

    //************************************************
    //***************** MEASURE_AND_CALIBRATE_MOTORS *
    case MEASURE_AND_CALIBRATE_MOTORS:
     currentOpModeName = "MESURE MOT.";
     measureAndCalibrateMotors();
     break;
    
    default:
     standby(); //safest thing to do
  }  
}

void free_run(void) {
 updateDisplayAndIndicators();
}

void setGPIOs(void) {
 //debug we might use this later
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // set all the motor control pins to outputs
  pinMode(ENA_L_PIN, OUTPUT);
  pinMode(ENB_R, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // set all encoder sensors to INPUT
  pinMode(S1motorEncoder_L_PIN,INPUT);
  pinMode(S2motorEncoder_L_PIN,INPUT);
  pinMode(S1motorEncoder_R_PIN,INPUT);
  pinMode(S2motorEncoder_R_PIN,INPUT);

 // indicators, controls and displays
  pinMode(MESSAGE_BUFFER_OVERFLOW_LED_PIN, OUTPUT);
  digitalWrite(MESSAGE_BUFFER_OVERFLOW_LED_PIN, LOW);

  pinMode(PS2X_CS, OUTPUT); //PS2 controler chip select pin
}

void standby(void) {
  updateDisplayAndIndicators(); // update display and indicators
  delay(50);
  
}

// put part of a string into char char_buffer
void strToChar(String s) {
  memset(char_buffer, 0, sizeof(char_buffer));
  for (byte i=0; i < s.length(); i++) {
    char_buffer[i] = s.charAt(i);
  }
}

void updateDisplayAndIndicators(void) {
 if ( (displayTimer + displayInterval) < millis()) {
   if (currentTopic == maxNbrTopics) {
     digitalWrite(MESSAGE_BUFFER_OVERFLOW_LED_PIN, HIGH);
   } else {digitalWrite(MESSAGE_BUFFER_OVERFLOW_LED_PIN, LOW);}

   // display topicsOverflow on HMI **************
   //debug
   topicsOverflow = random(100); //debug
   
   //page0.show(); // show status page
   if (topicsOverflow > 70) {
    jbuffstat.Set_font_color_pco(63488); // red  
   }
   else {
    jbuffstat.Set_font_color_pco(2016); // green
   }
   
   if (topicsOverflow > 100) { topicsOverflow = 100; }
   jbuffstat.setValue(topicsOverflow);
 
   // display current opMode on HMI **************
   memset(char_buffer, 0, sizeof(char_buffer));
   Serial.print("currentOpModeName= ");
   Serial.println(currentOpModeName);
   strToChar(currentOpModeName); // mode name converted to chr in char_buffer
   topmode.setText(char_buffer);
   displayTimer = millis();
 }
}

void setup()
{
  //debug noInterrupts(); //no interrupts at this point
  
  Serial.begin(9600);    // for debuging
  delay(500);
  Serial1.begin(115200); // for XBee for telemetry
  delay(500);
  Serial2.begin(38400); // for HMI communication
  delay(500);

  setGPIOs();
  motorAllStop();
  
  attachInterrupt(S1motorEncoder_L_PIN, ISR_S1_L, CHANGE);
  attachInterrupt(S1motorEncoder_R_PIN, ISR_S1_R, CHANGE);

  //setup timer interrupt for motor encoders
  encoderTimer.setPeriod(ENCODER_MEASURE_INTERVAL); // in microseconds
  encoderTimer.attachInterrupt(ISR_timerEncoder);
  encoderTimer.start();

/*
  //debug interrupts(); // allow interrupt starting here
  // debug
  // create somme messages
  //createTopic(byte tID, String dat, char typ, byte prio)
  //1
  currentTopic = storeTopic(CURRENT_OP_MODE, String(SENSORS_DEVELOPEMENT), 'b', 2);
  //2
  delay(3500);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "5", 'b', 1);
  //3
  delay(500);
  currentTopic = storeTopic(RMOTOR_SPEED, String(7824), 'u', 4);
  //4
  delay(50);
  currentTopic = storeTopic(LMOTOR_SPEED, String(5824), 'u', 4);
  //5
  delay(15);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "4", 'b', 1);
  //6
  delay(102);
  currentTopic = storeTopic(RMOTOR_SPEED, String(3824), 'u', 4);
  //7
  delay(152);
  currentTopic = storeTopic(LMOTOR_SPEED, String(3823), 'u', 4);
  //8 
  delay(254);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "0", 'b', 1);
  //9
  delay(82);
  currentTopic = storeTopic(LMOTOR_SPEED, String(11823), 'u', 4);
  //10
  delay(204);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "0", 'b', 1);
 //11 overflow 1
  delay(24);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "1", 'b', 1);
  //12 overflow 2
  delay(43);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "7", 'b', 1);
  //13 overflow 3
  delay(82);
  currentTopic = storeTopic(LMOTOR_SPEED, String(11823), 'u', 4);
  //14 overflow 4
  delay(204);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "0", 'b', 1);
  //15 overflow 5
  delay(24);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "1", 'b', 1);
  //16 overflow 6
  delay(43);
  currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "7", 'b', 1);
*/
 if (currentTopic == maxNbrTopics) {
  Serial.println("maxNbrTopics reached! Lost " + String(topicsOverflow) + " topics.");
  digitalWrite(MESSAGE_BUFFER_OVERFLOW_LED_PIN, HIGH);
 } else {digitalWrite(MESSAGE_BUFFER_OVERFLOW_LED_PIN, LOW);}

/*
 delay(5000);
 //send telemetry and clear buffer
 sendTelemetry(SERIAL_TELEMETRY_PORT);

 // then create more topics and print them
 
 Serial.println("\nTwo more\n");
 delay(1000);
 currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "0", 'b', 1);
 delay(5);
 currentTopic = storeTopic(COLLISION_SENSOR_STATUS, "7", 'b', 1);
 //send telemetry and clear buffer
 sendTelemetry(SERIAL_TELEMETRY_PORT);

 Serial.println("\nThere should be nothing after this\n");
 delay(1000);
*/
/*
 Serial.println(topicsRingBuffer[currentTopic].topicID);
 Serial.println(topicsRingBuffer[currentTopic].stc);
 Serial.println(topicsRingBuffer[currentTopic].dataType);
 Serial.println(topicsRingBuffer[currentTopic].priority);
 Serial.println();
*/  

 nexInit();
 delay(500);
 /* attach all pop event callback functions to components. */
 //*****attach
 bstat.attachPop(bstatPopCallback, &bstat);
 btele.attachPop(btelePopCallback, &btele);
 bsby.attachPop(bsbyPopCallback, &bsby);
 bopmode.attachPop(bopmodePopCallback, &bopmode);
 bfrun.attachPop(bfrunPopCallback, &bfrun);
 currentOpMode = STANDBY;
 page1.show();

 Serial.println("Setup finished\n");
}

void loop()
{
 //****************************************************************
 //***************** get new opMode value from command channel ****
 //****************************************************************
 // for now hard code the opMode

 //currentOpMode = TELEOP;
 //currentOpMode = MEASURE_AND_CALIBRATE_MOTORS;
 //currentOpMode = JUST_DO_THIS;
 //currentOpMode = STANDBY;
 //currentOpMode = SENSORS_DEVELOPEMENT;
 //currentOpMode = NADDOCAM;

 nexLoop(nex_listen_list);
 currentOpMode = selectedOpMode;
 runOpMode(currentOpMode);
 /*
  * if (opModeChangeRequested) {
  manageOpModeTimer();
 } else {
   runOpMode(currentOpMode);
 }
 */
}
//endif for COMPILE
#endif

