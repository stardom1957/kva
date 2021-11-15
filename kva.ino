/*
History, see kva_history.h tab
 */

// ************** Compile directives
#ifndef COMPILE_MAIN
#define COMPILE_MAIN
#endif

//#define DUE_TIMER_TEST0 // developpement test only, see DueTimer0 tab
//#define DUE_TIMER_TEST1 // developpement test only, see DueTimer1 tab
//#define COMPILE_PS2EXAMPLE // developpement test only
#define RTC_COMPILE

//************************************************

#ifdef COMPILE_MAIN
#define SERIAL_DEBUG_PORT     0 // serial port for debuging
#define SERIAL_TELEMETRY_PORT 1 // serial port to XBee RF module
#define SERIAL_HMI            2 // serial port to Nextion HMI
#define TELEMETRY               // compile telemetry code

#include <DueTimer.h>
#include <PS2X_lib.h>  //revised library from KurtE from Github

#ifdef TELEMETRY
#include "telemetry.h"
#endif

#include "kva.h"
#include "kva_rtc.h"
#include "kva_hmi.h" // for HMI display and control

void updateDisplayAndIndicators(void) {
 if ((millis() - displayTimer) > displayInterval) { // display interval
     //#TODO set SYSTEM_READY_LED according to system status
     //check RTC status
     //check other status
     //yellow LED on if any adverse condition occur

     if (!rtcFound || !hmiFound) digitalWrite(YELLOW_ALERT_CONDITION, HIGH);
     
    /* #TODO what to do with these
     !rtcInitialized
     rtcFound
    */
   
   // update message overflow LED on vehicule
/*
   #ifdef TELEMETRY
   if (currentTopic == maxNbrTopics) {
     digitalWrite(YELLOW_ALERT_CONDITION, HIGH);
   } else {digitalWrite(YELLOW_ALERT_CONDITION, LOW);}
   #endif
*/
   // display status according to currently selected status page
   switch (currentHMIpage) {
    case 0:
      // display topicsOverflow on page 0 on HMI **************
      //debug
      #ifdef TELEMETRY
      topicsOverflow = random(100); //debug
      if (topicsOverflow > 70) {
       jbuffstat.Set_font_color_pco(RED);
      }
      else {
        jbuffstat.Set_font_color_pco(GREEN); // green
      }
     
      if (topicsOverflow > 100) { topicsOverflow = 100; }
      jbuffstat.setValue(topicsOverflow);
     #endif
     
     // display current opMode on HMI **************
     memset(char_buffer, 0, sizeof(char_buffer));
     strToChar(currentOpModeName); // mode name converted to chr in char_buffer
     topmode.setText(char_buffer);

    break;

    case 1:
      // #TODO: ???
    break;

    default:
    break;
   }
   displayTimer = millis();
 } // check display timer
}


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

/* this function implement the TELEOP mode: the vehicule movement is fully controled by a remote operator using a Ps2 type controler. 
 *  This mode doesn't provide for any type of collision avoidance.

*/

//***************** PS2 controler setting function
void set_ps2x(void) {
 digitalWrite(PS2X_CS, LOW); // select controler for action
 //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for PS2_config_result
 PS2_config_result = ps2x.config_gamepad(SPI_CLK, SPI_MOSI, PS2X_CS, SPI_MISO, true, true);
 
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
    int emergency_slow {50};
    //slow forward
    if(ps2x.ButtonPressed(PSB_PAD_UP)){
      motorLeftSet(emergency_slow, FORWARD);
      motorRightSet(emergency_slow, FORWARD);
    }
    if(ps2x.ButtonReleased(PSB_PAD_UP)){
      motorAllStop();
    }

    //slow reverse
    if(ps2x.ButtonPressed(PSB_PAD_DOWN)){
      motorLeftSet(emergency_slow, REVERSE);
      motorRightSet(emergency_slow, REVERSE);
    }   
    if(ps2x.ButtonReleased(PSB_PAD_DOWN)){
      motorAllStop();
    }

    //slow left in place retation    
    if(ps2x.ButtonPressed(PSB_PAD_LEFT)){
      vehiculeRotateLeft(emergency_slow);
    }
    if(ps2x.ButtonReleased(PSB_PAD_LEFT)) {
      motorAllStop();
    }

    //slow in place right rotation
    if(ps2x.ButtonPressed(PSB_PAD_RIGHT)){
       vehiculeRotateRight(emergency_slow);
    }
    if(ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
      motorAllStop();
    }

  
  //**************** JOYSTICK OPERATION ********************************************
  //****************** all buttons are read only if button PSB_L1 is held pressed
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

//********************************************
//*********** RUN_PRESET_COURSE *******************

void run_preset_course(void) {
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

//debug code SHOULD BE DISABLED IN PRODUCTION
//********************************************
//**MEASURE_AND_CALIBRATE_MOTORS**************

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

void standby(void) {
  // #TODO 
  /*  make sure motors are stopped
   *  get status of Nextion display
   *  get status of RTC
   *  - get time values from RTC
   *  get status of SD card adaptor
   *  get status of IR sensor array
   *  get status of IMU
   *  - get attitude date from IMU
   *  get status of message buffer
   *  get status of serial comm to remote HMI
   *  
   *  send relevant telemetry
   * 
  */
}

void free_run(void) {
 // #TODO
 /*
  * get status of IR array
  * get obstacle
 */
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

    case RUN_PRESET_COURSE:
     currentOpModeName = "RUN_PRESET_COURSE";
     run_preset_course();
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
// put part of a string into char char_buffer
void strToChar(String s) {
  memset(char_buffer, 0, sizeof(char_buffer));
  for (byte i=0; i < s.length(); i++) {
    char_buffer[i] = s.charAt(i);
  }
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
  pinMode(YELLOW_ALERT_CONDITION, OUTPUT);
  pinMode(SYSTEM_READY_LED, OUTPUT);
  pinMode(PS2X_CS, OUTPUT); //PS2 controler chip select pin
  digitalWrite(SYSTEM_READY_LED, LOW);  // system not ready
  digitalWrite(YELLOW_ALERT_CONDITION, LOW); // yellow LED will indicate start of setup
}


void setup()
{
  Serial.begin(9600);    // for debuging, handled by Nextion library see Nextion.h
  Serial1.begin(115200); // XBee for telemetry
  Serial2.begin(115200); // HMI communication

  setGPIOs();
  motorAllStop();
  
  attachInterrupt(S1motorEncoder_L_PIN, ISR_S1_L, CHANGE);
  attachInterrupt(S1motorEncoder_R_PIN, ISR_S1_R, CHANGE);

  //setup timer interrupt for motor encoders
  encoderTimer.setPeriod(ENCODER_MEASURE_INTERVAL); // in microseconds
  encoderTimer.attachInterrupt(ISR_timerEncoder);
  encoderTimer.start();

  hmiFound = nexInit(); // start HMI

  /* attach all event callback functions to HMI components. */

  bstat1.attachPop(bstat1PopCallback, &bstat1);
  bstat2.attachPop(bstat2PopCallback, &bstat2);
  bop0.attachPop(bop0PopCallback, &bop0);
  bop2.attachPop(bop2PopCallback, &bop2);
  btele.attachPop(btelePopCallback, &btele);
  bsby.attachPop(bsbyPopCallback, &bsby);
  bfrun.attachPop(bfrunPopCallback, &bfrun);
  brtc0.attachPop(brtc0PopCallback, &brtc0);
  brtc1.attachPop(brtc1PopCallback, &brtc1);
  bplus5.attachPop(bplus5PopCallback, &bplus5);
  bplus1.attachPop(bplus1PopCallback, &bplus1);
  bminus.attachPop(bminusPopCallback, &bminus);
  bsetRTC.attachPop(bsetRTCPopCallback, &bsetRTC);

  /* this is the default opmode at start of vehicule */
  currentOpMode = STANDBY;

  #ifdef RTC_COMPILE
  kva_rtc_init(); // starts RTC
  #endif

  //debug Serial.println("Setup finished\n");
  digitalWrite(YELLOW_ALERT_CONDITION, LOW); //debug to indicate end of init
  digitalWrite(SYSTEM_READY_LED, HIGH); // now system is ready
}

void loop()
{
 updateDisplayAndIndicators();
 nexLoop(nex_listen_list);
 manageOpModeChange();
 runOpMode(currentOpMode);
 delay(10); //#TODO for developement. this delay will have to be ajusted later
}
//endif for COMPILE_MAIN
#endif

